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

#include "des/Time.h"

namespace des {

class Component;

// This defines an event handler function pointer.
class Event;
typedef void (Component::*EventHandler)(Event*);

// This is the base class for all events.
class Event {
 public:
  Event();
  Event(Component* _component, EventHandler _handler);
  Event(Component* _component, EventHandler _handler, Time _time);
  virtual ~Event();

  Component* component;
  EventHandler handler;
  Time time;

  // DO NOT USE. This is only used by the simulation core
  std::atomic<Event*> next;
};

// This defines a comparator object for comparing events.
class EventComparator {
 public:
  bool operator()(const Event* _lhs, const Event* _rhs) const;
};

// This is a simple template class to contain a single item in an event
template <typename T>
class ItemEvent : public Event {
 public:
  ItemEvent();
  explicit ItemEvent(T _item);
  ItemEvent(Component* _component, EventHandler _handler);
  ItemEvent(Component* _component, EventHandler _handler, T _item);
  ItemEvent(Component* _component, EventHandler _handler, Time _time);
  ItemEvent(Component* _component, EventHandler _handler, Time _time, T _item);
  ~ItemEvent();
  T item;
};

#include "Event.tcc"

}  // namespace des

#endif  // DES_EVENT_H_
