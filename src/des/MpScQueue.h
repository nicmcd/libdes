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
#ifndef DES_MPSCQUEUE_H_
#define DES_MPSCQUEUE_H_

#include <atomic>

#include "des/cacheline_util.h"
#include "des/Event.h"

namespace des {

// this is a multi-producer single-consumer event queue
class MpScQueue {
 public:
  MpScQueue();
  ~MpScQueue();

  // multiple threads can safely call this
  //  this pushes the event to the back of the queue
  void push(Event* _event);

  // only a single thread can safely call this
  //  this pops and returns the Event at the front of the queue
  //  (or nullptr if the queue is empty)
  Event* pop();

 private:
  // this points to the current tail
  //  this is cacheline aligned and padded
  alignas(CACHELINE_SIZE) std::atomic<Event*> tail_;
  char padding1_[CLPAD(sizeof(tail_))];

  // misc state variables
  Event dummies_[2];
  bool epoch_;  // indicates which dummy is in use
  Event* nextPop_;
};

}  // namespace des

#endif  // DES_MPSCQUEUE_H_
