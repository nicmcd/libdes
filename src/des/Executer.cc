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
#include <thread>

#include "des/Simulator.h"

namespace des {

Executer::Executer(Simulator* _simulator, u32 _id)
    : simulator_(_simulator), id_(_id), stop_(false), running_(false),
      executing_(false), executed_(0) {}

Executer::~Executer() {}

void Executer::start() {
  stop_ = false;
  if (!running_) {
    std::thread thread(&Executer::run, this);
    thread.detach();
  }
}

void Executer::stop() {
  stop_ = true;
}

bool Executer::running() const {
  return running_;
}

void Executer::addEvent(Event* _event) {
  queueLock_.lock();
  queue_.push(_event);
  queueLock_.unlock();
}

Executer::QueueStats Executer::queueStats() {
  Executer::QueueStats qs;
  Event* event = nullptr;
  qs.nextTime = Time();   ////////////////////// this is already the case?

  queueLock_.lock();
  qs.size = queue_.size();
  if (qs.size > 0) {
    event = queue_.top();
  }
  queueLock_.unlock();

  if (event != nullptr) {
    qs.nextTime = event->time;
  }

  return qs;
}

void Executer::execute() {
  assert(!executing_);
  executing_ = true;
}

bool Executer::executing() const {
  return executing_;
}

u64 Executer::executed() const {
  return executed_;
}

void Executer::run() {
  // running FSM
  assert(!running_);
  running_ = true;
  // printf("Executer %u running\n", id_);

  // loop forever (until 'stop' command is given)
  while (true) {
    // determine if we are allowed to execute
    if (executing_) {
      // get a current snapshot of time
      Time time = simulator_->time();

      // loop until all events up to the current time have been executed
      // printf("%u now executing at %s\n", id_, time.toString().c_str());
      while (true) {
        Event* event = nullptr;

        // peek at the next event
        queueLock_.lock();
        if (!queue_.empty()) {
          event = queue_.top();
        }
        queueLock_.unlock();

        // determine if this time step is complete
        if ((event == nullptr) || (event->time > time)) {
          // printf("%u done for now\n", id_);
          executing_ = false;
          break;
        }

        // get the next event, which is now guaranteed to be in this time step
        queueLock_.lock();
        event = queue_.top();
        queue_.pop();
        queueLock_.unlock();
        assert(event->time == time);

        // execute the event
        // printf("%u executing event at %s\n", id_, time.toString().c_str());
        Model* model = event->model;
        EventHandler handler = event->handler;
        (model->*handler)(event);
        executed_++;
      }
    }

    // determine if stop command has been given
    if (stop_) {
      break;
    } else {
      // do nothing
    }
  }

  // running FSM
  // printf("Executer %u stopping\n", id_);
  running_ = false;
  stop_ = false;
}

}  // namespace des
