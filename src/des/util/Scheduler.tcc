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
#ifndef DES_UTIL_SCHEDULER_H_
#error "Do not include this .tcc file directly, use the .h file instead"
#else

#include <cassert>

namespace des {

template <typename E>
Scheduler<E>::Scheduler(
    ActiveComponent* _component, EventHandler _handler)
    : event_(_component, _handler) {
  counter_.store(0, std::memory_order_release);
}

template <typename E>
Scheduler<E>::~Scheduler() {}

template <typename E>
E* Scheduler<E>::event() const {
  return &event_;
}

template <typename E>
bool Scheduler<E>::schedule(Time _time) const {
  assert(_time.valid());

  // do a fetch-n-add to ensure the event is scheduled
  u32 prev = counter_.fetch_add(1, std::memory_order_release);
  if (prev == 0) {
    // configure the event
    event_.time = _time;

    // schedule the event
    event_.component->simulator->addEvent(&event_);

    return true;
  }
  return false;
}

template <typename E>
void Scheduler<E>::executed() const {
  counter_.store(0, std::memory_order_release);
}

}  // namespace des

#endif  // DES_UTIL_SCHEDULER_H_
