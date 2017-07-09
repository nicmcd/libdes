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
#include "des/MpScQueue.h"

#include <cassert>

namespace des {

MpScQueue::MpScQueue() {
  // use the first dummy as the head
  epoch_ = false;
  dummies_[epoch_].next.store(nullptr, std::memory_order_release);

  // set the tail to the head
  tail_.store(&dummies_[epoch_], std::memory_order_release);

  // make the next to-be-popped event null
  nextPop_ = nullptr;
}

MpScQueue::~MpScQueue() {
  // make sure everything looks good
  assert(dummies_[epoch_].next == nullptr);
  assert(nextPop_ == nullptr);
  assert(tail_ == &dummies_[epoch_]);
}

void MpScQueue::push(Event* _event) {
  // have this event point to null
  _event->next.store(nullptr, std::memory_order_release);

  // set this event as the tail
  Event* oldTail = tail_.exchange(_event, std::memory_order_acquire);

  // point the old tail to the new tail
  assert(oldTail->next == nullptr);
  oldTail->next.store(_event, std::memory_order_release);
}

Event* MpScQueue::pop() {
  if (nextPop_ == nullptr) {
    // check if the list is empty
    Event* retEvent = dummies_[epoch_].next.load(std::memory_order_acquire);
    if (retEvent != nullptr) {
      // insert the other dummy into the tail
      push(&dummies_[!epoch_]);

      // wait until retEvent points to something
      while (!(nextPop_ = retEvent->next.load(std::memory_order_acquire))) {
        // make this spin-wait more efficient
        asm volatile("pause\n": : :"memory");
      }
      assert(nextPop_ != nullptr);

      // check if it points to the other dummy
      if (nextPop_ == &dummies_[!epoch_]) {
        // set nextPop_ to null and switch epochs
        nextPop_ = nullptr;
        epoch_ = !epoch_;
      }
    }
    // return the event (nullptr if list was empty)
    return retEvent;
  } else {
    // a previous call to this function generated some events that haven't
    //  been fully consumed, just pull from the list

    // we will eventually return nextPop_
    Event* retEvent = nextPop_;

    // wait until the event points to something
    Event* next;
    while (!(next = nextPop_->next.load(std::memory_order_acquire))) {
      // make this spin-wait more efficient
      asm volatile("pause\n": : :"memory");
    }

    // if nextPop_ is now the dummy, then the end of the poppable list has
    //  been reached, start over with null
    if (next == &dummies_[!epoch_]) {
      // start over with null
      nextPop_ = nullptr;

      // switch epochs
      epoch_ = !epoch_;
    } else {
      // transfer next to what the current next points to
      nextPop_ = next;
    }

    // return the event
    return retEvent;
  }
}

}  // namespace des
