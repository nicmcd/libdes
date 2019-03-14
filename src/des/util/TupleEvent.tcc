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
#ifndef DES_UTIL_TUPLEEVENT_TCC_
#define DES_UTIL_TUPLEEVENT_TCC_

#ifndef DES_UTIL_TUPLEEVENT_H_
#error "Do not include this .tcc file directly, use the .h file instead"
#else  // DES_UTIL_TUPLEEVENT_H_

#include <tuple>

namespace des {

template <typename... Types>
TupleEvent<Types...>::TupleEvent()
    : Event() {}

template <typename... Types>
template <typename Dummy>
TupleEvent<Types...>::TupleEvent(
    const Types&... _types)
    : Event(), tuple(_types...) {}

template <typename... Types>
TupleEvent<Types...>::TupleEvent(
    ActiveComponent* _component, EventHandler _handler)
    : Event(_component, _handler) {}

template <typename... Types>
template <typename Dummy>
TupleEvent<Types...>::TupleEvent(
    ActiveComponent* _component, EventHandler _handler,
    const Types&... _types)
    : Event(_component, _handler), tuple(_types...) {}

template <typename... Types>
TupleEvent<Types...>::TupleEvent(
    ActiveComponent* _component, EventHandler _handler,
    Time _time)
    : Event(_component, _handler, _time) {}

template <typename... Types>
template <typename Dummy>
TupleEvent<Types...>::TupleEvent(
    ActiveComponent* _component, EventHandler _handler,
    Time _time, const Types&... _types)
    : Event(_component, _handler, _time), tuple(_types...) {}

template <typename... Types>
TupleEvent<Types...>::~TupleEvent() {}

template <typename... Types>
template <std::size_t Index>
typename std::tuple_element<Index, std::tuple<Types...> >::type&
TupleEvent<Types...>::get() {
  return std::get<Index>(tuple);
}

template <typename... Types>
template <std::size_t Index>
void TupleEvent<Types...>::set(const typename std::tuple_element<
                               Index, std::tuple<Types...> >::type& _v) {
  std::get<Index>(tuple) = _v;
}

template <typename... Types>
void TupleEvent<Types...>::set(const Types&... _types) {
  tuple = std::make_tuple(_types...);
}

}  // namespace des

#endif  // DES_UTIL_TUPLEEVENT_H_
#endif  // DES_UTIL_TUPLEEVENT_TCC_
