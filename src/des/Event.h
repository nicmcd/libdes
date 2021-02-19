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
#ifndef DES_EVENT_H_
#define DES_EVENT_H_

#include <atomic>
#include <functional>

#include "des/Time.h"

namespace des {

class ActiveComponent;

using EventHandler = std::function<void()>;

class Event {
 private:
  Event();

 public:
  explicit Event(ActiveComponent* _component);
  Event(ActiveComponent* _component, EventHandler _handler);
  Event(ActiveComponent* _component, EventHandler _handler, Time _time);
  Event(ActiveComponent* _component, EventHandler _handler, Time _time,
        bool _clean);
  // Event(Event&& _event) = default;
  ~Event() = default;
  Event& operator=(const Event& _event);

  u32 executer() const;
  bool enqueued() const;

  EventHandler handler;
  Time time;
  bool skip;
  bool clean;

 private:
  bool enqueued_;
  u32 executer_;
  std::atomic<Event*> next_;

  friend class Simulator;  // for access to enqueued_, executer_
  friend class MpScQueue;  // for access to private constructor, next_
};

// This defines a comparator object for comparing events.
class EventComparator {
 public:
  bool operator()(const Event* _lhs, const Event* _rhs) const;
};

}  // namespace des

#endif  // DES_EVENT_H_
