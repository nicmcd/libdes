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
    : Simulator(0) {}

Simulator::Simulator(u32 _numThreads)
    : time_(0, 0), logger_(nullptr) {
  if (_numThreads == 0) {
    _numThreads = std::thread::hardware_concurrency() - 1;
    if (_numThreads == 0) {
      _numThreads = 1;
    }
  }

  if (_numThreads + 1 > std::thread::hardware_concurrency()) {
    printf("*************************************************************\n"
           "* WARNING WARNING WARNING WARNING WARNING WARNING WARNING   *\n"
           "* The simulator will have terrible performance if there are *\n"
           "* more execution threads than the hardware supports.        *\n"
           "*************************************************************\n");
  }

  executers_.resize(_numThreads);
  for (u32 id = 0; id < _numThreads; id++) {
    std::get<0>(executers_.at(id)) = new Executer(this, id);
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

void Simulator::simulate() {
  // start all executers
  for (auto e : executers_) {
    Executer* exe = std::get<0>(e);
    exe->start();
  }

  // statistics tracking
  u64 totalEvents = 0;
  u64 intervalEvents = 0;
  Time lastSimTime = 0;
  std::chrono::steady_clock::time_point startTime =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point lastRealTime = startTime;
  std::chrono::duration<f64> sum(0);

  // loop forever (until there are no more events in any executer queue)
  while (true) {
    // find the event next time (also get other stats needed)
    Time nextTime = Time();
    u64 eventCount = 0;
    for (auto& e : executers_) {
      // gather queue stats from executer
      Executer* exe = std::get<0>(e);
      Executer::QueueStats& qs = std::get<1>(e);
      qs = exe->queueStats();

      // minimize next event time
      if (qs.size > 0) {
        eventCount = qs.size;
        if (qs.nextTime < nextTime) {
          nextTime = qs.nextTime;
        }
      }
    }

    // determine if simulation is complete (no more events)
    if (eventCount == 0) {
      break;
    }

    // update the time
    time_ = nextTime;

    // turn on executers with events for this time
    for (auto& e : executers_) {
      Executer* exe = std::get<0>(e);
      Executer::QueueStats& qs = std::get<1>(e);
      if (qs.nextTime == nextTime) {
        exe->execute();
      }
    }

    // wait for all executers to finish
    for (auto& e : executers_) {
      Executer* exe = std::get<0>(e);
      while (exe->executing()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }
    }
  }

  // give executers stop command
  for (auto e : executers_) {
    Executer* exe = std::get<0>(e);
    exe->stop();
  }

  // wait for all executers to finish
  for (auto e : executers_) {
    Executer* exe = std::get<0>(e);
    while (exe->running()) {
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}


  /*
  // all these variables are used for statistics gathering and printing
  u64 totalEvents = 0;
  u64 intervalEvents = 0;
  u64 lastSimTime = 0;
  std::chrono::steady_clock::time_point startTime =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point lastRealTime = startTime;
  std::chrono::duration<f64> sum(0);

  while (queue_.empty() == false) {
    // pop next event
    Event* event = queue_.top();
    time_ = event->time;
    epsilon_ = event->epsilon;
    queue_.pop();

    // call the models event handler
    Model* model = event->model;
    EventHandler handler = event->handler;
    (model->*handler)(event);

    // count events
    totalEvents++;
    intervalEvents++;

    // if requested, show stats
    if (showStats_) {
      showStats_ = false;

      std::chrono::steady_clock::time_point realTime =
          std::chrono::steady_clock::now();
      f64 elapsedRealTime =
          std::chrono::duration_cast<std::chrono::duration<f64>>(
              realTime - lastRealTime).count();

      u64 elapsedSimTime = time_ - lastSimTime;
      lastSimTime = time_;

      printf("%11lu events : %8lu elements : %12lu units : %10.0f events/sec "
             ": %4.2f events/unit : %8.0f units/sec\n",
             totalEvents,
             queue_.size(),
             time_,
             intervalEvents / elapsedRealTime,
             intervalEvents / static_cast<f64>(elapsedSimTime),
             elapsedSimTime / static_cast<f64>(elapsedRealTime));

      lastRealTime = realTime;
      intervalEvents = 0;
    }
  }

  if (_printStatsSummary) {
    // print statistic totals
    std::chrono::steady_clock::time_point realTime =
        std::chrono::steady_clock::now();
    std::chrono::duration<f64> totalElapsedRealTime =
        std::chrono::duration_cast<std::chrono::duration<f64>>(
            realTime - startTime);
    f64 runTime = totalElapsedRealTime.count();

    printf("\n"
           "Total event count:      %lu\n"
           "Total simulation units: %lu\n"
           "Total real seconds:     %.3f\n"
           "\n"
           "Events per real seconds:    %.3f\n"
           "Events per sim units:       %.3f\n"
           "Sim units per real seconds: %.3f\n"
           "\n",
           totalEvents, time_, runTime, totalEvents / runTime,
           totalEvents / static_cast<f64>(time_), time_ / runTime);
  }
  */

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

}  // namespace des
