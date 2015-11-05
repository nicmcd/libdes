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

#include "des/Event.h"
#include "des/Simulator.h"

namespace des {

Executer::Executer(Simulator* _simulator)
    : simulator_(_simulator), stop_(false), running_(false) {}

Executer::~Executer() {}

void Executer::start() {
  stop_ = false;
  if (!running_) {
    std::thread thread(&Executer::run, this);
  }
}

void Executer::stop() {
  stop_ = true;
  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  stop_ = false;
}

bool Executer::running() const {
  return running_;
}

void Executer::addEvent(Event* _event) {
  u64 tid = std::hash<std::thread::id>(std::this_thread::get_id());
  printf("%lu adding event\n", tid);
}

void Executer::run() {
  assert(!running_);
  running_ = true;

  printf("%lu running\n", std::this_thread::get_id().hash());

  running_ = false;
}

}  // namespace des
