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

#include <cassert>
#include <cstdio>
#include <ctime>

#include <algorithm>
#include <ratio>
#include <thread>

#include "des/Event.h"
#include "des/Component.h"
#include "des/Mapper.h"
#include "des/Observer.h"

namespace des {

// these variables are read/write variables that are thread local
//  these are cache line aligned and padded individually
struct alignas(CACHELINE_SIZE) ExecuterState {
  // this is the executer id using for executer-level multiplexing
  u32 id;
  char padding1[CLPAD(sizeof(id))];

  // this is used during execution to find the minimum next time step
  TimeStep minTimeStep;
  char padding2[CLPAD(sizeof(minTimeStep))];
};

// this creates a thread local version of ExecuterState for each executer
static thread_local ExecuterState exeState;


Simulator::Simulator()
    : Simulator(1, nullptr) {}

Simulator::Simulator(u32 _numExecuters, Mapper* _mapper)
    : numExecuters_(_numExecuters), mapper_(_mapper) {
  // check inputs
  assert(numExecuters_ > 0);
  assert(numExecuters_ == 1 || _mapper);

  // create the event queues
  queueSets_ = new QueueSet[numExecuters_ + 1];  // +1 for tail padding

  // initialize the state
  state_.running.store(false, std::memory_order_release);
  state_.timeStep = 0;

  // initialize the stats
  stats_.eventCount.store(0, std::memory_order_release);
  stats_.timeStepCount = 0;
  stats_.lastEventCount = 0;
  stats_.lastTick = 0;

  // observer defaults
  observingInterval_ = 1.0;
  observingMask_ = 1 < 10;
}

Simulator::~Simulator() {
  delete[] queueSets_;
}

u32 Simulator::executers() const {
  return numExecuters_;
}

Time Simulator::time() const {
  return Time::create(state_.timeStep);
}

void Simulator::setLogger(Logger* _logger) {
  logger_ = _logger;
}

Logger* Simulator::getLogger() const {
  return logger_;
}

void Simulator::addObserver(Observer* _observer) {
  observers_.push_back(_observer);
}

void Simulator::setObservingIntervel(f64 _interval) {
  observingInterval_ = _interval;
}

void Simulator::setObservingPower(u64 _expPow2Events) {
  observingMask_ = 1 << _expPow2Events;
}

void Simulator::addComponent(Component* _component) {
  // duplicate name detection
  if (!components_.insert(std::make_pair(_component->fullname(),
                                         _component)).second) {
    fprintf(stderr, "duplicate component name detected: %s\n",
            _component->fullname().c_str());
    assert(false);
  }

  // set debug if needed
  if (toBeDebugged_.count(_component->fullname()) == 1) {
    _component->debug = true;
    u64 res = toBeDebugged_.erase(_component->fullname());
    (void)res;
    assert(res == 1);
  }

  // executer mapping
  if (numExecuters_ == 1) {
    _component->executer_ = 0;
  } else {
    _component->executer_ = mapper_->map(numExecuters_, _component);
    assert(_component->executer_ < numExecuters_);
  }
}

Component* Simulator::getComponent(const std::string& _fullname) const {
  return components_.at(_fullname);
}

void Simulator::removeComponent(const std::string& _fullname) {
  components_.at(_fullname)->executer_ = U32_MAX;
  u64 res = components_.erase(_fullname);
  (void)res;
  assert(res == 1);
}

u64 Simulator::numComponents() const {
  return components_.size();
}

void Simulator::addDebugName(const std::string& _fullname) {
  bool res = toBeDebugged_.insert(_fullname).second;
  (void)res;
  assert(res);
}

void Simulator::debugCheck() {
  // ensure that all Components that were marked to be debugged got accounted
  //  for during component construction
  for (auto it = toBeDebugged_.begin(); it != toBeDebugged_.end(); ++it) {
    fprintf(stderr, "invalid component name: %s\n", it->c_str());
  }
  assert(toBeDebugged_.size() == 0);
  toBeDebugged_.reserve(0);
}

void Simulator::addEvent(Event* _event) {
  TimeStep eventTimeStep = _event->time.raw();

  // verify time
  TimeStep timeStep = state_.timeStep;
  assert(eventTimeStep > timeStep);

  // push the event into the queue
  u32 id = _event->component->executer_;
  if (id != exeState.id) {
    // this event is crossing executers, put it into the oqueue
    MpScQueue& oqueue = queueSets_[id].oqueue;
    oqueue.push(_event);
  } else {
    // this event is NOT crossing executers, put it directly into the pqueue
    EventQueue& pqueue = queueSets_[id].pqueue;
    pqueue.push(_event);
  }

  // minimum time step comparison
  exeState.minTimeStep = std::min(exeState.minTimeStep, eventTimeStep);
}

void Simulator::simulate() {
  // set running to true
  state_.running.store(true, std::memory_order_release);

  // set the epoch to false(0)
  state_.epoch.store(false, std::memory_order_release);

  // set the yetToArrive to numExecuters_
  state_.yetToArrive.store(numExecuters_, std::memory_order_release);

  // find the first time to simulate
  TimeStep firstTimeStep = TIMESTEP_INV;
  for (u32 id = 0; id < numExecuters_; id++) {
    MpScQueue& oqueue = queueSets_[id].oqueue;
    EventQueue& pqueue = queueSets_[id].pqueue;

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

  // set the timeStep
  state_.timeStep = firstTimeStep;

  // set the nextTimeStep to start simulation
  state_.nextTimeStep[0].store(firstTimeStep, std::memory_order_release);
  state_.nextTimeStep[1].store(TIMESTEP_INV, std::memory_order_release);

  // create threads for executers and run
  std::vector<std::thread> threads;
  for (u32 id = 0; id < numExecuters_; id++) {
    threads.push_back(std::thread(&Simulator::executer, this, id));
  }

  // track the amount of time in simulation
  std::chrono::steady_clock::time_point startTime =
      std::chrono::steady_clock::now();
  stats_.startRealTime = startTime;
  stats_.lastRealTime = startTime;

  // now wait for all executer threads to complete
  for (u32 id = 0; id < numExecuters_; id++) {
    threads.at(id).join();
  }

  // give a report to the observers
  if (observers_.size() > 0) {
    Observer::SummaryStatistics stats;

    stats.eventCount = stats_.eventCount.load(std::memory_order_acquire);
    stats.timeSteps = stats_.timeStepCount;
    stats.ticks = time().tick();
    std::chrono::steady_clock::time_point realTime =
        std::chrono::steady_clock::now();
    std::chrono::duration<f64> totalElapsedRealTime =
        std::chrono::duration_cast<std::chrono::duration<f64>>(
            realTime - startTime);
    stats.seconds = totalElapsedRealTime.count();

    for (Observer* observer : observers_) {
      observer->summaryStatistics(stats);
    }
  }
}

u32 Simulator::executerId() const {
  return exeState.id;
}

void Simulator::executer(u32 _id) {
  // set up the thread local executer id
  exeState.id = _id;

  // get a reference to the event queues
  MpScQueue& oqueue = queueSets_[_id].oqueue;
  EventQueue& pqueue = queueSets_[_id].pqueue;

  // this is the current epoch
  bool cEpoch = state_.epoch.load(std::memory_order_acquire);

  // loop until specifically told not to
  while (state_.running.load(std::memory_order_acquire)) {
    // get the current time of execution
    TimeStep cTimeStep = state_.nextTimeStep[cEpoch].load(
        std::memory_order_acquire);

    // execute events for this timestep, track minimum timestep of new events
    exeState.minTimeStep = TIMESTEP_INV;
    // loop until all events up to the current time have been executed
    u64 executed = 0;
    TimeStep queueMinTimeStep;
    while (true) {
      bool done = false;
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
        if (queueMinTimeStep == cTimeStep) {
          // the queue is not empty and the top event can be executed, continue
          pqueue.pop();
        } else {
          // the queue is not empty but the event is not for now, done
          done = true;
        }
      } else {
        // the queue is empty, done
        queueMinTimeStep = TIMESTEP_INV;
        done = true;
      }
      if (done) {
        break;
      }

      // execute the event
      Component* component = event->component;
      EventHandler handler = event->handler;
      (component->*handler)(event);
      executed++;
    }

    // update the executed counter
    stats_.eventCount.fetch_add(executed, std::memory_order_release);

    // take the minimum of the new minimum and queue minimum
    //  Note: others are accessing this queue in parallel but this is OK because
    //  they'll also see new minimums
    assert(queueMinTimeStep > cTimeStep);
    TimeStep minTimeStep = std::min(exeState.minTimeStep, queueMinTimeStep);

    // publish the minimum if we have a lower minimum
    TimeStep currMin;
    while (minTimeStep < (currMin = state_.nextTimeStep[!cEpoch].load(
               std::memory_order_acquire))) {
      if (state_.nextTimeStep[!cEpoch].compare_exchange_weak(
              currMin, minTimeStep, std::memory_order_release,
              std::memory_order_acquire)) {
        break;
      }
    }

    // announce arrival to the barrier
    if ((state_.yetToArrive--) == 1) {
      // last arrival at the barrier, setup the next epoch

      // increment the timeStep counter
      stats_.timeStepCount++;

      // set the next time
      TimeStep nextTimeStep = state_.nextTimeStep[!cEpoch].load(
          std::memory_order_acquire);
      bool done = nextTimeStep == TIMESTEP_INV;
      if (done) {
        nextTimeStep = time().nextEpsilon().raw();
      }
      state_.timeStep = nextTimeStep;

      // show progress statistics
      if (observers_.size() > 0) {
        // use the event mask to reduce the amount of time retrieving
        u64 eventCount = stats_.eventCount.load(std::memory_order_acquire);
        if ((eventCount & observingMask_) == 0) {
          // get the time to see if some progress observering is needed
          std::chrono::steady_clock::time_point realTime =
              std::chrono::steady_clock::now();
          std::chrono::duration<f64> elapsedRealTime =
              std::chrono::duration_cast<std::chrono::duration<f64>>(
                  realTime - stats_.lastRealTime);
          f64 elapsedTime = elapsedRealTime.count();
          if (elapsedTime >= observingInterval_) {
            // gather and compute all needed statistics
            u64 tick = time().tick();
            u64 intervalEventCount = eventCount - stats_.lastEventCount;
            u64 intervalTick = tick - stats_.lastTick;
            std::chrono::duration<f64> totalRealTime =
                std::chrono::duration_cast<std::chrono::duration<f64>>(
                    realTime - stats_.startRealTime);
            f64 totalTime = totalRealTime.count();

            // give a report to the observers
            Observer::ProgressStatistics stats;
            stats.seconds = totalTime;
            stats.eventCount = eventCount;
            stats.ticks = tick + 1;
            stats.eventsPerSecond = intervalEventCount / elapsedTime;
            stats.ticksPerSecond = intervalTick / elapsedTime;
            for (Observer* observer : observers_) {
              observer->progressStatistics(stats);
            }

            // save lasts for next time
            stats_.lastRealTime = realTime;
            stats_.lastEventCount = eventCount;
            stats_.lastTick = tick;
          }
        }
      }

      // reset the next time step
      state_.nextTimeStep[cEpoch].store(
          TIMESTEP_INV, std::memory_order_release);

      // reset yetToArrive
      state_.yetToArrive.store(numExecuters_, std::memory_order_release);

      // check if the simulation is complete
      if (done) {
        state_.running.store(false, std::memory_order_release);
      }

      // switch to the next epoch
      state_.epoch.store(!cEpoch, std::memory_order_release);
    } else {
      // not the last arrival, just wait
      while (state_.epoch.load(std::memory_order_acquire) == cEpoch) {
        // make this spin-wait more efficient
        asm volatile("pause\n": : :"memory");
      }
    }

    // change to the next epoch
    cEpoch = !cEpoch;
  }
}

}  // namespace des
