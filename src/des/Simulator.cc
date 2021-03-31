/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * - Neither the name of prim nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "des/Simulator.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <ratio>   // NOLINT
#include <thread>  // NOLINT
#include <utility>

#include "des/ActiveComponent.h"
#include "des/Component.h"
#include "des/Event.h"
#include "des/Mapper.h"
#include "des/Observer.h"

namespace des {

// these variables are read/write variables that are thread local
struct alignas(CACHELINE_SIZE) ExecuterState {
 public:
  ExecuterState() : id(U32_MAX), currTimeStep(0) {}

  // initial padding isn't needed for thread_local variables :)

  // this is the executer id using for executer-level multiplexing
  u32 id;

  // this is the current time step
  TimeStep currTimeStep;

  // this is used during execution to find the minimum next time step
  TimeStep minTimeStep;

  // this is executer view of the current epoch
  bool epoch;

  // this is a local pointer to all executer MinTime arrays
  std::vector<Simulator::MinTime*> minTimeArrays;

  // ensure no contention across threads
  char padding4[CLPAD(sizeof(id) + sizeof(currTimeStep) + sizeof(minTimeStep) +
                      sizeof(epoch) + sizeof(minTimeArrays))];
};

// this creates a thread local version of ExecuterState for each executer
static thread_local ExecuterState exeState;

// This is a special TimeStep value that is not TIMESTEP_INV and is also
//  not a valid TimeStep. This is used by the executers to indicate that they
//  don't have any events for the future. Executers exit their execution if this
//  becomes the next time step to execute/
static const u64 TIMESTEP_INF = TICK_INV << EPSILON_BITS;
static_assert(TIMESTEP_INF != TIMESTEP_INV, "bad TIMESTEP_INF");

Simulator::Simulator() : Simulator(std::thread::hardware_concurrency()) {}

Simulator::Simulator(u32 _numExecuters)
    : numExecuters_(_numExecuters), logger_(nullptr), mapper_(nullptr) {
  // check inputs
  assert(numExecuters_ > 0);

  // check TIMESTEP_INF is invalid time
  assert(!Time::create(TIMESTEP_INF).valid());

  // create the event queues
  executerSets_ = new ExecuterSet[numExecuters_ + 1];  // +1 for tail padding

  // initialize the state
  running_ = false;

  // initialize the stats
  stats_.eventCount.store(0, std::memory_order_release);
  stats_.timeStepCount = 0;
  stats_.lastEventCount = 0;
  stats_.lastTimeStepCount = 0;
  stats_.lastTick = 0;

  // observer defaults
  observeProgress_ = false;
  observingInterval_ = 1.0;
  observingMask_ = (1 << 10) - 1;

  // calculate the number of iterations in the barrier
  barrierIterations_ = static_cast<u32>(ceil(log2(numExecuters_)));
}

Simulator::~Simulator() {
  assert(components_.size() == 0);
  delete[] executerSets_;
}

u32 Simulator::executers() const {
  return numExecuters_;
}

Time Simulator::time() const {
  return Time::create(exeState.currTimeStep);
}

void Simulator::setMapper(Mapper* _mapper) {
  mapper_ = _mapper;
}

Mapper* Simulator::getMapper() const {
  return mapper_;
}

void Simulator::setLogger(Logger* _logger) {
  logger_ = _logger;
}

Logger* Simulator::getLogger() const {
  return logger_;
}

void Simulator::addObserver(Observer* _observer) {
  observers_.push_back(_observer);
  if (!std::isnan(observingInterval_)) {
    observeProgress_ = true;
  }
}

void Simulator::setObservingInterval(f64 _interval) {
  assert(std::isnan(_interval) || _interval > 0.0);
  observingInterval_ = _interval;
  if (std::isnan(observingInterval_)) {
    observeProgress_ = false;
  } else {
    observeProgress_ = observers_.size() > 0;
  }
}

void Simulator::setObservingPower(u32 _expPow2Events) {
  assert(_expPow2Events < 64);
  observingMask_ = 1 << (u64)_expPow2Events;
}

void Simulator::addComponent(Component* _component) {
  // duplicate name detection
  if (!components_.insert(std::make_pair(_component->fullname(), _component))
           .second) {
    fprintf(stderr, "duplicate component name detected: %s\n",
            _component->fullname().c_str());
    assert(false);
  }

  // set debug if needed
  if (toBeDebugged_.count(_component->fullname()) == 1) {
    _component->debug = true;
    u64 res = toBeDebugged_.erase(_component->fullname());
    assert(res == 1);
  }
}

void Simulator::mapComponent(ActiveComponent* _component) {
  // executer mapping
  if (numExecuters_ == 1) {
    _component->executer_ = 0;
  } else {
    assert(mapper_);
    _component->executer_ = mapper_->map(numExecuters_, _component);
    assert(_component->executer_ < numExecuters_);
  }
}

Component* Simulator::getComponent(const std::string& _fullname) const {
  return components_.at(_fullname);
}

void Simulator::removeComponent(const std::string& _fullname) {
  u64 res = components_.erase(_fullname);
  assert(res == 1);
}

u64 Simulator::numComponents() const {
  return components_.size();
}

void Simulator::addDebugName(const std::string& _fullname) {
  bool res = toBeDebugged_.insert(_fullname).second;
  assert(res);
}

void Simulator::debugNameCheck() {
  // ensure that all Components that were marked to be debugged got accounted
  //  for during component construction
  for (auto it = toBeDebugged_.begin(); it != toBeDebugged_.end(); ++it) {
    fprintf(stderr, "invalid component name: %s\n", it->c_str());
  }
  assert(toBeDebugged_.size() == 0);
  toBeDebugged_.reserve(0);
}

u32 Simulator::addClock(Tick _period, Tick _phase) {
  assert(_period > 0);
  u32 clockId = clocks_.size();
  clocks_.push_back(std::make_pair(_period, _phase));
  return clockId;
}

Tick Simulator::clockPeriod(u32 _clockId) const {
  return clocks_.at(_clockId).first;
}

Tick Simulator::clockPhase(u32 _clockId) const {
  return clocks_.at(_clockId).second;
}

u64 Simulator::cycle(u32 _clockId) const {
  const std::pair<Tick, Tick>& clock = clocks_.at(_clockId);
  return (time().tick() + clock.second) / clock.first;
}

u64 Simulator::cycles(u32 _clockId, Time _time) const {
  return _time.tick() / clockPeriod(_clockId);
}

bool Simulator::isCycle(u32 _clockId) const {
  const std::pair<Tick, Tick>& clock = clocks_.at(_clockId);
  Tick tick = time().tick();
  Tick rem = tick % clock.first;
  return rem == clock.second;
}

Time Simulator::futureCycle(u32 _clockId, u32 _cycles) const {
  return futureCycle(_clockId, _cycles, 0);
}

Time Simulator::futureCycle(u32 _clockId, u32 _cycles, Epsilon _epsilon) const {
  const std::pair<Tick, Tick>& clock = clocks_.at(_clockId);
  assert(_cycles > 0);
  Tick tick = time().tick();
  Tick rem = tick % clock.first;
  if (rem != clock.second) {
    tick += (clock.second - rem);
    if (rem > clock.second) {
      tick += clock.first;
    }
    _cycles--;
  }
  return Time(tick + (clock.first * _cycles), _epsilon);
}

void Simulator::addEvent(Event* _event) const {
  assert(_event->time.valid());
  _event->enqueued_ = true;
  TimeStep eventTimeStep = _event->time.raw();

  // verify event time is in the future
  if (running_) {
    assert(eventTimeStep > exeState.currTimeStep);
  }

  // push the event into the queue
  u32 id = _event->executer_;
  if (id != exeState.id) {
    // this event is crossing executers, put it into the oqueue
    MpScQueue& oqueue = executerSets_[id].oqueue;
    oqueue.push(_event);
  } else {
    // this event is NOT crossing executers, put it directly into the pqueue
    EventQueue& pqueue = executerSets_[id].pqueue;
    pqueue.push(_event);
  }

  // minimum time step comparison
  exeState.minTimeStep = std::min(exeState.minTimeStep, eventTimeStep);
}

void Simulator::simulate() {
  assert(!running_);

  // Initializes all components.
  for (auto& [name, component] : components_) {
    component->initialize();
  }

  // set running to true
  running_ = true;

  // find the first time to simulate
  TimeStep firstTimeStep = TIMESTEP_INF;
  for (u32 id = 0; id < numExecuters_; id++) {
    MpScQueue& oqueue = executerSets_[id].oqueue;
    EventQueue& pqueue = executerSets_[id].pqueue;

    // transfer from oqueue to pqueue
    {
      Event* tmpEvt;
      while ((tmpEvt = oqueue.pop())) {
        pqueue.push(tmpEvt);
      }
    }

    // check time
    if (!pqueue.empty()) {
      TimeStep queueMinTimeStep = pqueue.top()->time.raw();
      firstTimeStep = std::min(firstTimeStep, queueMinTimeStep);
    }
  }

  // take the current time
  std::chrono::steady_clock::time_point startTime =
      std::chrono::steady_clock::now();

  // make a vector for min time steps
  std::vector<Simulator::MinTime*> minTimeArrays(numExecuters_, nullptr);

  // only attempt to run if needed
  if (firstTimeStep < TIMESTEP_INF) {
    // initialize the barrier
    barrierInit(firstTimeStep);

    // create threads for executers and run
    std::vector<std::thread> threads;
    for (u32 id = 0; id < numExecuters_; id++) {
      threads.push_back(
          std::thread(&Simulator::executer, this, id, &minTimeArrays));
    }

    // track the amount of time in simulation
    stats_.startRealTime = startTime;
    stats_.lastRealTime = startTime;

    // now wait for all executer threads to complete
    for (u32 id = 0; id < numExecuters_; id++) {
      threads.at(id).join();
    }

    // clean up the MinTime arrays
    u32 slots = 1 + 2 * barrierIterations_;
    for (Simulator::MinTime* array : minTimeArrays) {
      assert(array);
      numa_free(array, slots * sizeof(Simulator::MinTime));
    }
  }

  // set running to false
  running_ = false;

  // copy the reported time to thread local info
  exeState.currTimeStep = timeStep_;

  // give a report to the observers
  if (observers_.size() > 0) {
    Observer::SummaryStatistics stats;

    stats.eventCount = stats_.eventCount.load(std::memory_order_acquire);
    stats.timeSteps = stats_.timeStepCount;
    stats.ticks = time().tick();
    std::chrono::steady_clock::time_point realTime =
        std::chrono::steady_clock::now();
    std::chrono::duration<f64> totalElapsedRealTime =
        std::chrono::duration_cast<std::chrono::duration<f64>>(realTime -
                                                               startTime);
    stats.seconds = totalElapsedRealTime.count();

    for (Observer* observer : observers_) {
      observer->summaryStatistics(stats);
    }
  }

  // Finalizes all components.
  for (auto& [name, component] : components_) {
    component->finalize();
  }
}

void Simulator::seed(u64 _seed) {
  for (u32 e = 0; e < numExecuters_; e++) {
    executerSets_[e].random.seed(_seed + e);
  }
}

rnd::Random* Simulator::random() {
  return &executerSets_[executerId()].random;
}

u32 Simulator::executerId() const {
  return exeState.id;
}

void Simulator::executer(u32 _id,
                         std::vector<Simulator::MinTime*>* _minTimeArrays) {
  // init the barrier for this executer
  barrierExecuterInit(_id, _minTimeArrays);

  // get a reference to the event queues
  MpScQueue& oqueue = executerSets_[_id].oqueue;
  EventQueue& pqueue = executerSets_[_id].pqueue;

  // get the original time step
  exeState.currTimeStep = timeStep_;

  // loop forever
  while (true) {
    // execute events for this timestep, track minimum timestep of new events
    exeState.minTimeStep = TIMESTEP_INF;

    // loop until all events up to the current time have been executed
    u64 executed = 0;
    TimeStep queueMinTimeStep;
    while (true) {
      Event* event;

      // transfer events from oqueue to pqueue
      {
        Event* tmpEvt;
        while ((tmpEvt = oqueue.pop())) {
          pqueue.push(tmpEvt);
        }
      }

      // determine if this time step is complete
      //  if not, get the next event
      if (!pqueue.empty()) {
        event = pqueue.top();
        queueMinTimeStep = event->time.raw();
        if (queueMinTimeStep == exeState.currTimeStep) {
          // the queue is not empty and the top event can be executed, continue
          pqueue.pop();
        } else {
          // the queue is not empty but the event is not for now, done
          break;
        }
      } else {
        // the queue is empty, done
        queueMinTimeStep = TIMESTEP_INF;
        break;
      }

      // Executes the event if not marked with skip.
      // Marks the event as not enqueued and optionally deletes it.
      if (!event->skip) {
        event->handler();
        executed++;
      }
      event->enqueued_ = false;
      if (event->clean) {
        delete event;
      }
    }

    // take the minimum of the new minimum and queue minimum
    //  Note: others are accessing this queue in parallel but this is OK because
    //  they'll also see new minimums
    assert(queueMinTimeStep > exeState.currTimeStep);
    TimeStep minTimeStep = std::min(exeState.minTimeStep, queueMinTimeStep);

    // hit the barrier, get the new time step to execute
    exeState.currTimeStep = barrier(_id, minTimeStep, executed);

    // break the execution loop when done
    if (exeState.currTimeStep == TIMESTEP_INF) {
      break;
    }
  }
}

void Simulator::barrierInit(TimeStep _firstTimeStep) {
  // set the time
  timeStep_ = _firstTimeStep;

  // set the yetToArrive to numExecuters_
  state_.yetToArrive.store(numExecuters_, std::memory_order_release);
}

void Simulator::barrierExecuterInit(
    u32 _id, std::vector<Simulator::MinTime*>* _minTimeArrays) {
  // set up the thread local executer id
  exeState.id = _id;

  // always start on epoch false(0)
  //  this is used in the barrier
  exeState.epoch = false;

  // allocate and initialize the MinTime array for this thread
  const u32 slots = 1 + 2 * barrierIterations_;
  void* buff = numa_alloc_local(slots * sizeof(Simulator::MinTime));
  assert(buff != NULL);
  Simulator::MinTime* minTimeArray = static_cast<Simulator::MinTime*>(buff);
  for (u32 mt = 0; mt < slots; mt++) {
    minTimeArray[mt].minTimeStep.store(TIMESTEP_INV, std::memory_order_release);
  }

  // publish a pointer to the array
  _minTimeArrays->at(_id) = minTimeArray;

  // announce that this thread's MinTime array is set
  state_.yetToArrive--;

  // wait until every thread has set their MinTime array
  while (state_.yetToArrive.load(std::memory_order_acquire) != 0) {}

  // copy all the MinTime array pointers to thread local variables
  exeState.minTimeArrays.resize(numExecuters_, nullptr);
  for (u32 exe = 0; exe < numExecuters_; exe++) {
    // retrieve
    Simulator::MinTime* minTimeArray = _minTimeArrays->at(exe);
    assert(minTimeArray);

    // set local
    exeState.minTimeArrays.at(exe) = minTimeArray;
  }
}

TimeStep Simulator::barrier(u32 _id, TimeStep _minTimeStep, u64 _executed) {
  TimeStep minTimeStep = _minTimeStep;

  // update the executed counter
  if (_executed > 0) {
    stats_.eventCount.fetch_add(_executed, std::memory_order_release);
  }

  if (numExecuters_ > 1) {
    for (u32 iter = 0; iter < barrierIterations_; iter++) {
      u32 dist = 1 << iter;                     // pow(2, iter)
      u32 peer = (_id + dist) % numExecuters_;  // TODO(nic): remove divide
      assert(peer != _id);

      u32 index = 1 + (u32)exeState.epoch * barrierIterations_ + iter;
      assert(index < (1 + 2 * barrierIterations_));
      Simulator::MinTime* thisArray = exeState.minTimeArrays.at(_id);
      Simulator::MinTime* peerArray = exeState.minTimeArrays.at(peer);

      // set minTimeStep to peer thread
      peerArray[index].minTimeStep.store(minTimeStep,
                                         std::memory_order_release);

      // wait for this minTimeStep to not be TIMESTEP_INV
      TimeStep minTimeStepOther;
      while ((minTimeStepOther = thisArray[index].minTimeStep.load(
                  std::memory_order_acquire)) == TIMESTEP_INV) {}

      // set minTimeStep back to TIMESTEP_INV
      thisArray[index].minTimeStep.store(TIMESTEP_INV,
                                         std::memory_order_release);

      // re-minimize what this thread believes as minimum
      minTimeStep = std::min(minTimeStep, minTimeStepOther);
    }
  }

  // executer N-1 will record statistics
  if (_id == (numExecuters_ - 1)) {
    // increment the timeStep counter
    stats_.timeStepCount++;

    // set the next time globally if simulation is over
    if (minTimeStep == TIMESTEP_INF) {
      timeStep_ = Time::create(exeState.currTimeStep).nextEpsilon().raw();
    }

    // show progress statistics
    if (observeProgress_) {
      // use the event mask to reduce the amount of time retrieving
      if ((stats_.timeStepCount & observingMask_) == 0) {
        // get the time to see if some progress observing is needed
        std::chrono::steady_clock::time_point realTime =
            std::chrono::steady_clock::now();
        std::chrono::duration<f64> elapsedRealTime =
            std::chrono::duration_cast<std::chrono::duration<f64>>(
                realTime - stats_.lastRealTime);
        f64 elapsedTime = elapsedRealTime.count();
        if (elapsedTime >= observingInterval_) {
          // gather and compute all needed statistics
          Time cTime = Time::create(exeState.currTimeStep);
          Tick tick = cTime.tick();
          u64 eventCount = stats_.eventCount.load(std::memory_order_acquire);
          u64 intervalEventCount = eventCount - stats_.lastEventCount;
          u64 intervalTimeStepCount =
              stats_.timeStepCount - stats_.lastTimeStepCount;
          Tick intervalTick = tick - stats_.lastTick;
          std::chrono::duration<f64> totalRealTime =
              std::chrono::duration_cast<std::chrono::duration<f64>>(
                  realTime - stats_.startRealTime);
          f64 totalTime = totalRealTime.count();

          // give a report to the observers
          Observer::ProgressStatistics stats;
          stats.seconds = totalTime;
          stats.eventCount = eventCount;
          stats.ticks = tick;
          stats.eventsPerSecond = intervalEventCount / elapsedTime;
          stats.ticksPerSecond = intervalTick / elapsedTime;
          stats.stepsPerSecond = intervalTimeStepCount / elapsedTime;
          for (Observer* observer : observers_) {
            observer->progressStatistics(stats);
          }

          // save lasts for next time
          stats_.lastRealTime = realTime;
          stats_.lastEventCount = eventCount;
          stats_.lastTimeStepCount = stats_.timeStepCount;
          stats_.lastTick = tick;
        }
      }
    }
  }

  // change to the next epoch
  exeState.epoch = !exeState.epoch;

  // return the next time to execute
  assert(minTimeStep <= _minTimeStep);
  assert(minTimeStep < TIMESTEP_INV);
  return minTimeStep;
}

}  // namespace des
