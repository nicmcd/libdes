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
#include "des/Event.h"

#include <cassert>

#include "des/ActiveComponent.h"
#include "des/Time.h"
#include "prim/prim.h"

namespace des {

Event::Event()
    : handler(nullptr),
      time(),
      skip(false),
      clean(false),
      enqueued_(false),
      executer_(U32_MAX),
      next_(nullptr) {}

Event::Event(ActiveComponent* _component)
    : handler(nullptr),
      time(),
      skip(false),
      clean(false),
      enqueued_(false),
      executer_(_component->executer()),
      next_(nullptr) {}

Event::Event(ActiveComponent* _component, EventHandler _handler)
    : handler(_handler),
      time(),
      skip(false),
      clean(false),
      enqueued_(false),
      executer_(_component->executer()),
      next_(nullptr) {}

Event::Event(ActiveComponent* _component, EventHandler _handler, Time _time)
    : handler(_handler),
      time(_time),
      skip(false),
      clean(false),
      enqueued_(false),
      executer_(_component->executer()),
      next_(nullptr) {}

Event::Event(ActiveComponent* _component, EventHandler _handler, Time _time,
             bool _clean)
    : handler(_handler),
      time(_time),
      skip(false),
      clean(_clean),
      enqueued_(false),
      executer_(_component->executer()),
      next_(nullptr) {}

Event& Event::operator=(const Event& _event) {
  assert(!_event.enqueued_);
  handler = _event.handler;
  time = _event.time;
  skip = _event.skip;
  clean = _event.clean;
  enqueued_ = false;
  executer_ = _event.executer_;
  next_.store(nullptr, std::memory_order_release);
  return *this;
}

u32 Event::executer() const {
  return executer_;
}

bool Event::enqueued() const {
  return enqueued_;
}

bool EventComparator::operator()(const Event* _lhs, const Event* _rhs) const {
  return _lhs->time > _rhs->time;
}

}  // namespace des
