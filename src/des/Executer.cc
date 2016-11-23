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
#include "des/Executer.h"

#include <prim/prim.h>

#include <cassert>

#include <chrono>

#include "des/Simulator.h"

namespace des {

Executer::Executer(Simulator* _simulator, u32 _id, bool _direct)
    : simulator_(_simulator), id_(_id), direct_(_direct), stop_(false),
      running_(false), executing_(false), executed_(0) {}

Executer::~Executer() {}

void Executer::start() {
  assert(!direct_);
  assert(!running_);
  assert(!stop_);
  thread_ = std::thread(&Executer::run, this);
}

void Executer::stop() {
  assert(!direct_);
  stop_ = true;
  thread_.join();
  thread_ = std::thread();
}

bool Executer::running() const {
  assert(!direct_);
  return running_;
}

void Executer::addEvent(Event* _event) {
  queueLock_.lock();
  queue_.push(_event);
  queueLock_.unlock();
}

Executer::QueueStats Executer::queueStats() {
  assert(executing_ == false);  // this function isn't safe when executing
  Executer::QueueStats qs;
  Event* event = nullptr;
  qs.nextTime = Time();  // default to invalid time

  qs.size = queue_.size();
  if (qs.size > 0) {
    event = queue_.top();
    qs.nextTime = event->time;
  }

  return qs;
}

void Executer::execute() {
  if (!direct_) {
    assert(!executing_);
    executing_ = true;
  } else {
    timeStep();
  }
}

bool Executer::executing() const {
  assert(!direct_);
  return executing_;
}

u64 Executer::executed() const {
  return executed_;
}

void Executer::run() {
  assert(!direct_);

  // running FSM
  assert(!running_);
  running_ = true;
  // printf("Executer %u running\n", id_);

  // loop forever (until 'stop' command is given)
  while (true) {
    // determine if we are allowed to execute
    if (executing_) {
      timeStep();
    }

    // stop when commanded to
    if (stop_) {
      break;
    }

    // make this spin-wait more efficient
    asm volatile("pause\n": : :"memory");
  }

  // running FSM
  // printf("Executer %u stopping\n", id_);
  stop_ = false;
  running_ = false;
}

void Executer::timeStep() {
  // get a current snapshot of time
  Time time = simulator_->time();

  // loop until all events up to the current time have been executed
  // printf("%u now executing at %s\n", id_, time.toString().c_str());
  while (true) {
    bool done = false;
    Event* event = nullptr;

    // determine if this time step is complete
    //  if not, get the next event
    queueLock_.lock();
    if (!queue_.empty()) {
      event = queue_.top();
    }
    if ((event == nullptr) || (event->time > time)) {
      executing_ = false;
      done = true;
    } else {
      queue_.pop();
    }
    queueLock_.unlock();

    if (done) {
      break;
    }

    assert(event->time == time);

    // execute the event
    // printf("%u executing event at %s\n", id_, time.toString().c_str());
    Model* model = event->model;
    EventHandler handler = event->handler;
    (model->*handler)(event);
    executed_++;
  }
}

}  // namespace des
