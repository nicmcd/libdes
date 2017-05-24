/*
 * Copyright (c) 2012-2016, Nic McDonald
 * All rights reserved.
 *
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
#include "des/Logger.h"
#include "des/Model.h"

namespace des {

// these variables are read/write variables that are thread local
//  these are cache line aligned and padded individually
struct alignas(CACHELINE_SIZE) Executer {
  // this is the executer id using for thread-level multiplexing
  u32 id;
  char padding1[CLPAD(sizeof(id))];

  // this is used during execution to find the minimum next time step
  TimeStep minTimeStep;
  char padding2[CLPAD(sizeof(minTimeStep))];
};

static thread_local Executer exeState;


Simulator::Simulator()
    : Simulator(1) {}

Simulator::Simulator(u32 _numThreads)
    : logger_(nullptr) {
  // configure number of threads in this simulator
  u32 hwThreads = std::thread::hardware_concurrency();
  if (_numThreads == 0) {
    _numThreads = hwThreads;
    if (_numThreads == 0) {
      _numThreads = 1;
    }
  }
  if ((hwThreads > 0) && (_numThreads > hwThreads)) {
    fprintf(stderr,
            "*************************************************************\n"
            "* WARNING WARNING WARNING WARNING WARNING WARNING WARNING   *\n"
            "* The simulator will have terrible performance if there are *\n"
            "* more execution threads than the hardware supports.        *\n"
            "*************************************************************\n");
  }
  numThreads_ = _numThreads;

  // create the event queues
  queueSets_ = new QueueSet[numThreads_ + 1];  // +1 for tail padding

  // initialize the simState
  simState_.running.store(false, std::memory_order_release);
  simState_.timeStep = 0;
  simState_.showStats.store(false, std::memory_order_release);
  simState_.eventCount.store(0, std::memory_order_release);
  simState_.timeStepCount = 0;
  simState_.tickCount = 0;
}

Simulator::~Simulator() {}

u32 Simulator::threads() const {
  return numThreads_;
}

Time Simulator::time() const {
  return Time::create(simState_.timeStep);
}

void Simulator::addEvent(Event* _event) {
  TimeStep eventTimeStep = _event->time.raw();

  // verify time
  TimeStep timeStep = simState_.timeStep;
  assert(eventTimeStep > timeStep);

  // push the event into the queue
  u32 id = _event->model->executer;
  if (id != exeState.id) {
    // this event is crossing threads, put it into the oqueue
    MpScQueue& oqueue = queueSets_[id].oqueue;
    oqueue.push(_event);
  } else {
    // this event is NOT crossing threads, put it directly into the pqueue
    EventQueue& pqueue = queueSets_[id].pqueue;
    pqueue.push(_event);
  }

  // minimum time step comparison
  exeState.minTimeStep = std::min(exeState.minTimeStep, eventTimeStep);
}

void Simulator::simulate(bool _logSummary) {
  // set running to true
  simState_.running.store(true, std::memory_order_release);

  // set the epoch to false(0)
  simState_.epoch.store(false, std::memory_order_release);

  // set the yetToArrive to numThreads_
  simState_.yetToArrive.store(numThreads_, std::memory_order_release);

  // find the first time to simulate
  TimeStep firstTimeStep = TIMESTEP_INV;
  for (u32 id = 0; id < numThreads_; id++) {
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
  simState_.timeStep = firstTimeStep;

  // set the nextTimeStep to start simulation
  simState_.nextTimeStep[0].store(firstTimeStep, std::memory_order_release);
  simState_.nextTimeStep[1].store(TIMESTEP_INV, std::memory_order_release);

  // create threads for executers and run
  std::vector<std::thread> threads;
  for (u32 id = 0; id < numThreads_; id++) {
    threads.push_back(std::thread(&Simulator::executer, this, id));
  }

  // track the amount of time in simulation
  std::chrono::steady_clock::time_point startTime =
      std::chrono::steady_clock::now();

  // now wait for all executer threads to complete
  for (u32 id = 0; id < numThreads_; id++) {
    threads.at(id).join();
  }

  // optionally give a report
  if (logger_ && _logSummary) {
    // create a buffer to show stats on
    const u32 STATS_SIZE = 1024;
    char* statsString = new char[STATS_SIZE];

    u64 eventCount = simState_.eventCount.load(std::memory_order_acquire);
    u64 timeTicks = simState_.tickCount;
    u64 timeSteps = simState_.timeStepCount;

    // print statistic totals
    std::chrono::steady_clock::time_point realTime =
        std::chrono::steady_clock::now();
    std::chrono::duration<f64> totalElapsedRealTime =
        std::chrono::duration_cast<std::chrono::duration<f64>>(
            realTime - startTime);
    f64 runTime = totalElapsedRealTime.count();

    Tick ctick = time().tick();
    s32 r = snprintf(statsString, STATS_SIZE,
                     "\n"
                     "Total event count:          %lu\n"
                     "Total simulation ticks:     %lu\n"
                     "Total unique time steps:    %lu\n"
                     "Total real seconds:         %.3f\n"
                     "\n"
                     "Events per real seconds:    %.3f\n"
                     "Events per sim ticks:       %.3f\n"
                     "Sim ticks per real seconds: %.3f\n"
                     "\n",
                     eventCount, timeTicks, timeSteps, runTime,
                     eventCount / runTime,
                     eventCount / static_cast<f64>(ctick),
                     ctick / runTime);
    assert(r > 0 && r < (s32)STATS_SIZE);
    logger_->log(statsString, r);

    // clean up the stats string buffer
    delete[] statsString;
  }

  /*
  // create a buffer to show stats on
  const u32 STATS_SIZE = 1024;
  char* statsString = new char[STATS_SIZE];

  // start all executers (besides the direct executer)
  for (u32 id = 1; id < executers_.size(); id++) {
    executers_.at(id)->start();
  }

  // statistics tracking
  u64 uniqueTimeSteps = 0;
  u64 currEventCount = 0;
  u64 prevEventCount = 0;
  u64 intervalEvents = 0;
  Tick lastSimTicks = 0;
  std::chrono::steady_clock::time_point startTime =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point lastRealTime = startTime;
  std::chrono::duration<f64> sum(0);

  // loop forever (until there are no more events in any executer queue)
  while (true) {
    // find the event next time (also get other stats needed)
    currEventCount = 0;
    Time nextTime = Time();
    bool more = false;
    for (u32 id = 0; id < executers_.size(); id++) {
      // gather queue stats from executer
      Executer* exe = executers_.at(id);
      // Executer::QueueStats& qs = std::get<1>(e);
      // qs = exe->queueStats();
      currEventCount += exe->executed();

      // minimize next event time
      // if (qs.size > 0) {
      //   more = true;
      //   if (qs.nextTime < nextTime) {
      //     nextTime = qs.nextTime;
      //   }
      // }
    }

    // update the statistics
    u64 deltaEventCount = currEventCount - prevEventCount;
    intervalEvents += deltaEventCount;

    // determine if simulation is complete (no more events)
    if (!more) {
      break;
    }

    // count time steps
    uniqueTimeSteps++;

    // update the time
    time_ = nextTime;

    // turn on executers with events for this time (besides the direct executer)
    for (u32 id = 1; id < executers_.size(); id++) {
      Executer* exe = executers_.at(id);
      // Executer::QueueStats& qs = std::get<1>(e);
      // if (qs.nextTime == nextTime) {
      //   exe->execute();
      // }
    }

    // execute the direct executer in this thread
    {
      Executer* exe = executers_.at(0);
      // Executer::QueueStats& qs = std::get<1>(e);
      // if (qs.nextTime == nextTime) {
      //   exe->execute();
      // }
    }

    // optionally print statistics
    if (logger_ && showStats_) {
      showStats_ = false;

      std::chrono::steady_clock::time_point realTime =
          std::chrono::steady_clock::now();
      f64 elapsedRealTime =
          std::chrono::duration_cast<std::chrono::duration<f64> >(
              realTime - lastRealTime).count();

      Tick ctick = time_.tick();
      Tick elapsedSimTicks = ctick - lastSimTicks;

      s32 r = snprintf(statsString, STATS_SIZE,
                       "%11lu events : %12lu ticks : %10.0f events/sec "
                       ": %4.2f events/tick : %8.0f ticks/sec\n",
                       currEventCount,
                       ctick + 1,
                       intervalEvents / elapsedRealTime,
                       intervalEvents / static_cast<f64>(elapsedSimTicks),
                       elapsedSimTicks / static_cast<f64>(elapsedRealTime));
      assert(r > 0 && r < (s32)STATS_SIZE);
      logger_->log(statsString, r);

      lastSimTicks = ctick;
      lastRealTime = realTime;
      intervalEvents = 0;
    }

    // wait for all executers to finish (direct executer is already finished)
    for (u32 id = 1; id < executers_.size(); id++) {
      Executer* exe = executers_.at(id);
      while (exe->executing()) {
        // make this spin-wait more efficient
        asm volatile("pause\n": : :"memory");
      }
    }
  }

  // give executers stop command (besides the direct executer)
  for (u32 id = 1; id < executers_.size(); id++) {
    Executer* exe = executers_.at(id);
    exe->stop();
  }

  if (logger_ && _logSummary) {
    // print statistic totals
    std::chrono::steady_clock::time_point realTime =
        std::chrono::steady_clock::now();
    std::chrono::duration<f64> totalElapsedRealTime =
        std::chrono::duration_cast<std::chrono::duration<f64>>(
            realTime - startTime);
    f64 runTime = totalElapsedRealTime.count();

    Tick ctick = time_.tick();
    s32 r = snprintf(statsString, STATS_SIZE,
                     "\n"
                     "Total event count:          %lu\n"
                     "Total simulation ticks:     %lu\n"
                     "Total unique time steps:    %lu\n"
                     "Total real seconds:         %.3f\n"
                     "\n"
                     "Events per real seconds:    %.3f\n"
                     "Events per sim ticks:       %.3f\n"
                     "Sim ticks per real seconds: %.3f\n"
                     "\n",
                     currEventCount, ctick + 1, uniqueTimeSteps, runTime,
                     currEventCount / runTime,
                     currEventCount / static_cast<f64>(ctick),
                     ctick / runTime);
    assert(r > 0 && r < (s32)STATS_SIZE);
    logger_->log(statsString, r);
  }

  // clean up the stats string buffer
  delete[] statsString;
  */
}

Logger* Simulator::getLogger() const {
  return logger_;
}

void Simulator::setLogger(Logger* _logger) {
  logger_ = _logger;
}

void Simulator::addModel(Model* _model) {
  static u32 executer = 0;

  if (!models_.insert(std::make_pair(_model->fullName(), _model)).second) {
    fprintf(stderr, "duplicate component name detected: %s\n",
            _model->fullName().c_str());
    assert(false);
  }
  if (toBeDebugged_.count(_model->fullName()) == 1) {
    _model->debug = true;
    u64 res = toBeDebugged_.erase(_model->fullName());
    (void)res;
    assert(res == 1);
  }

  _model->executer = executer;
  executer = (executer + 1) % numThreads_;
}

Model* Simulator::getModel(const std::string& _fullName) const {
  return models_.at(_fullName);
}

void Simulator::removeModel(const std::string& _fullName) {
  models_.at(_fullName)->executer = U32_MAX;
  u64 res = models_.erase(_fullName);
  (void)res;
  assert(res == 1);
}

u64 Simulator::numModels() const {
  return models_.size();
}

void Simulator::addDebugName(const std::string& _fullName) {
  bool res = toBeDebugged_.insert(_fullName).second;
  (void)res;
  assert(res);
}

void Simulator::debugCheck() {
  // ensure that all Models that were marked to be debugged got accounted for
  for (auto it = toBeDebugged_.begin(); it != toBeDebugged_.end(); ++it) {
    fprintf(stderr, "%s is an invalid component name\n", it->c_str());
  }
  assert(toBeDebugged_.size() == 0);
  toBeDebugged_.reserve(0);
}

void Simulator::showStats() {
  simState_.showStats.store(true, std::memory_order_release);
}

u32 Simulator::threadId() const {
  return exeState.id;
}

void Simulator::executer(u32 _id) {
  // set up the thread local executer id
  exeState.id = _id;

  // get a reference to the event queues
  MpScQueue& oqueue = queueSets_[_id].oqueue;
  EventQueue& pqueue = queueSets_[_id].pqueue;

  // this is the current epoch
  bool cEpoch = simState_.epoch.load(std::memory_order_acquire);

  // loop until specifically told not to
  while (simState_.running.load(std::memory_order_acquire)) {
    // get the current time of execution
    TimeStep cTimeStep = simState_.nextTimeStep[cEpoch].load(
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
      Model* model = event->model;
      EventHandler handler = event->handler;
      (model->*handler)(event);
      executed++;
    }

    // update the executed counter
    simState_.eventCount.fetch_add(executed, std::memory_order_release);

    // take the minimum of the new minimum and queue minimum
    //  Note: others are accessing this queue in parallel but this is OK because
    //  they'll also see new minimums
    assert(queueMinTimeStep > cTimeStep);
    TimeStep minTimeStep = std::min(exeState.minTimeStep, queueMinTimeStep);

    // publish the minimum if we have a lower minimum
    TimeStep currMin;
    while (minTimeStep < (currMin = simState_.nextTimeStep[!cEpoch].load(
               std::memory_order_acquire))) {
      if (simState_.nextTimeStep[!cEpoch].compare_exchange_weak(
              currMin, minTimeStep, std::memory_order_release,
              std::memory_order_acquire)) {
        break;
      }
    }

    // announce arrival to the barrier
    if ((simState_.yetToArrive--) == 1) {
      // last arrival at the barrier, setup the next epoch

      // increment the timeStep counter
      simState_.timeStepCount++;

      // set the next time
      TimeStep nextTimeStep = simState_.nextTimeStep[!cEpoch].load(
          std::memory_order_acquire);
      bool done = nextTimeStep == TIMESTEP_INV;
      if (done) {
        nextTimeStep = time().nextEpsilon().raw();
      }
      bool newTick = nextTimeStep > simState_.timeStep;
      simState_.timeStep = nextTimeStep;

      // increment the tick counter
      if (newTick) {
        simState_.tickCount++;
      }

      // show statistics
      if (simState_.showStats.load(std::memory_order_acquire)) {
        // deactivate flag
        simState_.showStats.store(false, std::memory_order_release);

        // show the stats
      }

      // reset the next time step
      simState_.nextTimeStep[cEpoch].store(
          TIMESTEP_INV, std::memory_order_release);

      // reset yetToArrive
      simState_.yetToArrive.store(numThreads_, std::memory_order_release);

      // check if the simulation is complete
      if (done) {
        simState_.running.store(false, std::memory_order_release);
      }

      // switch to the next epoch
      simState_.epoch.store(!cEpoch, std::memory_order_release);
    } else {
      // not the last arrival, just wait
      while (simState_.epoch.load(std::memory_order_acquire) == cEpoch) {
        // make this spin-wait more efficient
        asm volatile("pause\n": : :"memory");
      }
    }

    // change to the next epoch
    cEpoch = !cEpoch;
  }
}

}  // namespace des
