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

Simulator::Simulator(u32 _numThreads)
    : time_(0), epsilon_(0), showStats_(false), logger_(nullptr) {
  u32 hwThreads = _numThreads;
  if (hwThreads == 0) {
    hwThreads = std::thread::hardware_concurrency();
  }
  assert(hwThreads > 0);
  assert(hwThreads <= 64);
  executers_.resize(hwThreads, NULL);
  for (auto& e : executers_) {
    e = new Executer(this);
  }
  working_ = 0;
}

Simulator::~Simulator() {
  for (auto& e : executers_) {
    delete e;
  }
}

u64 Simulator::time() const {
  return time_;
}

u8 Simulator::epsilon() const {
  return epsilon_;
}

void Simulator::addEvent(Event* _event) {
  assert((_event->time == time_) ?
         (_event->epsilon > epsilon_) :
         (_event->time > time_));
  u32 exe = _event->model->executer;
  working_.fetch_or(1 << exe);
  executers_.at(exe)->addEvent(_event);
}

u64 Simulator::numEvents() const {
  return queue_.size();
}

void Simulator::simulate(bool _printStatsSummary) {
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
}

void Simulator::showStats() {
  showStats_ = true;
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
  executer = (executer + 1) % executers_.size();
}

Model* Simulator::getModel(const std::string& _fullName) const {
  return models_.at(_fullName);
}

void Simulator::removeModel(const std::string& _fullName) {
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

bool Simulator::EventComparator::operator()(
    const Event* _lhs,
    const Event* _rhs) const {
  if (_lhs->time == _rhs->time) {
    return _lhs->epsilon > _rhs->epsilon;
  }
  return _lhs->time > _rhs->time;
}

}  // namespace des
