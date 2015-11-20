/*
 * Copyright (c) 2012-2015, Nic McDonald
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
 * - Neither the name of prim nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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

#include <chrono>
#include <ratio>
#include <thread>

#include "des/Event.h"
#include "des/Executer.h"
#include "des/Logger.h"
#include "des/Model.h"

namespace des {

Simulator::Simulator()
    : Simulator(1) {}

Simulator::Simulator(u32 _numThreads)
    : time_(0, 0), logger_(nullptr), showStats_(false) {
  if (_numThreads == 0) {
    _numThreads = std::thread::hardware_concurrency();
    if (_numThreads == 0) {
      _numThreads = 1;
    }
  }

  if (_numThreads > std::thread::hardware_concurrency()) {
    fprintf(stderr,
            "*************************************************************\n"
            "* WARNING WARNING WARNING WARNING WARNING WARNING WARNING   *\n"
            "* The simulator will have terrible performance if there are *\n"
            "* more execution threads than the hardware supports.        *\n"
            "*************************************************************\n");
  }

  executers_.resize(_numThreads);
  for (u32 id = 0; id < _numThreads; id++) {
    bool direct = id == 0;
    std::get<0>(executers_.at(id)) = new Executer(this, id, direct);
  }
  logger_ = new Logger();
}

Simulator::~Simulator() {
  for (auto& e : executers_) {
    delete std::get<0>(e);
  }
  delete logger_;
}

Time Simulator::time() const {
  return time_;
}

void Simulator::addEvent(Event* _event) {
  assert(_event->time >= time_);
  u32 exe = _event->model->executer;
  std::get<0>(executers_.at(exe))->addEvent(_event);
}

void Simulator::simulate(bool _logSummary) {
  // create a buffer to show stats on
  const u32 STATS_SIZE = 1024;
  char* statsString = new char[STATS_SIZE];

  // start all executers (besides the direct executer)
  for (u32 id = 1; id < executers_.size(); id++) {
    std::get<0>(executers_.at(id))->start();
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
      auto& e = executers_.at(id);
      Executer* exe = std::get<0>(e);
      Executer::QueueStats& qs = std::get<1>(e);
      qs = exe->queueStats();
      currEventCount += exe->executed();

      // minimize next event time
      if (qs.size > 0) {
        more = true;
        if (qs.nextTime < nextTime) {
          nextTime = qs.nextTime;
        }
      }
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
      auto& e = executers_.at(id);
      Executer* exe = std::get<0>(e);
      Executer::QueueStats& qs = std::get<1>(e);
      if (qs.nextTime == nextTime) {
        exe->execute();
      }
    }

    // execute the direct executer directly
    {
      auto& e = executers_.at(0);
      Executer* exe = std::get<0>(e);
      Executer::QueueStats& qs = std::get<1>(e);
      if (qs.nextTime == nextTime) {
        exe->execute();
      }
    }

    // optionally print statistics
    if (showStats_) {
      showStats_ = false;

      std::chrono::steady_clock::time_point realTime =
          std::chrono::steady_clock::now();
      f64 elapsedRealTime =
          std::chrono::duration_cast<std::chrono::duration<f64> >(
              realTime - lastRealTime).count();

      Tick elapsedSimTicks = time_.tick - lastSimTicks;

      s32 r = snprintf(statsString, STATS_SIZE,
                       "%11lu events : %12lu ticks : %10.0f events/sec "
                       ": %4.2f events/tick : %8.0f ticks/sec\n",
                       currEventCount,
                       time_.tick + 1,
                       intervalEvents / elapsedRealTime,
                       intervalEvents / static_cast<f64>(elapsedSimTicks),
                       elapsedSimTicks / static_cast<f64>(elapsedRealTime));
      assert(r > 0 && r < (s32)STATS_SIZE);
      logger_->log(statsString);

      lastSimTicks = time_.tick;
      lastRealTime = realTime;
      intervalEvents = 0;
    }

    // wait for all executers to finish (direct executer is already finished)
    for (u32 id = 1; id < executers_.size(); id++) {
      auto& e = executers_.at(id);
      Executer* exe = std::get<0>(e);
      while (exe->executing()) {}
    }
  }

  // give executers stop command (besides the direct executer)
  for (u32 id = 1; id < executers_.size(); id++) {
    auto& e = executers_.at(id);
    Executer* exe = std::get<0>(e);
    exe->stop();
  }

  if (_logSummary) {
    // print statistic totals
    std::chrono::steady_clock::time_point realTime =
        std::chrono::steady_clock::now();
    std::chrono::duration<f64> totalElapsedRealTime =
        std::chrono::duration_cast<std::chrono::duration<f64>>(
            realTime - startTime);
    f64 runTime = totalElapsedRealTime.count();

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
                     currEventCount, time_.tick + 1, uniqueTimeSteps, runTime,
                     currEventCount / runTime,
                     currEventCount / static_cast<f64>(time_.tick),
                     time_.tick / runTime);
    assert(r > 0 && r < (s32)STATS_SIZE);
    logger_->log(statsString);
  }

  // clean up the stats string buffer
  delete[] statsString;
}

Logger* Simulator::getLogger() const {
  return logger_;
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
  executer = (executer + 1) % executers_.size();
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
  showStats_ = true;
}

}  // namespace des
