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
#ifndef DES_EXECUTER_H_
#define DES_EXECUTER_H_

#include <prim/prim.h>

#include <queue>
#include <vector>

#include "des/Event.h"
#include "des/SpinLock.h"

namespace des {

class Simulator;

class Executer {
 public:
  struct QueueStats {
    u64 size;
    Time nextTime;
  };

  Executer(Simulator* _simulator, u32 _id, bool _direct);
  ~Executer();

  void start();
  void stop();
  bool running() const;
  void addEvent(Event* _event);
  QueueStats queueStats();
  void execute();
  bool executing() const;
  u64 executed() const;  // only safe to call during !executing()

 private:
  void run();
  void timeStep();

  Simulator* simulator_;
  u32 id_;
  bool direct_;

  volatile bool stop_;
  volatile bool running_;
  volatile bool executing_;

  std::priority_queue<Event*, std::vector<Event*>, EventComparator> queue_;
  SpinLock queueLock_;

  u64 executed_;
};

}  // namespace des

#endif  // DES_EXECUTER_H_
