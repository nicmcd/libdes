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
#ifndef DES_TUPLEEVENT_H_
#define DES_TUPLEEVENT_H_

#include <tuple>

#include "des/Event.h"
#include "des/Time.h"

namespace des {

class ActiveComponent;

// This is a simple template class to contain a single tuple in an event
template <typename... Types>
class TupleEvent : public Event {
 public:
  // initializes nothing
  TupleEvent();

  // initializes only the tuple
  template <typename Dummy = void>  // needed for <> case
  explicit TupleEvent(const Types&...);

  // initializes component and handler
  TupleEvent(ActiveComponent* _component, EventHandler _handler);

  // initializes component, handler, and tuple
  template <typename Dummy = void>  // needed for <> case
  TupleEvent(ActiveComponent* _component, EventHandler _handler,
             const Types&...);

  // initializes component, handler, and time
  TupleEvent(ActiveComponent* _component, EventHandler _handler,
             Time _time);

  // initializes component, handler, time, and tuple
  template <typename Dummy = void>  // needed for <> case
  TupleEvent(ActiveComponent* _component, EventHandler _handler,
             Time _time, const Types&...);

  ~TupleEvent();

  // gets an element from the tuple
  template <std::size_t Index>
  typename std::tuple_element<Index, std::tuple<Types...> >::type& get();

  // this is the actual tuple object
  //  most users won't need this
  std::tuple<Types...> tuple;
};

}  // namespace des

#include "TupleEvent.tcc"

#endif  // DES_TUPLEEVENT_H_
