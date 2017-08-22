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
#define DES_UTIL_SCHEDULER_H_

#include <atomic>

#include "des/ActiveComponent.h"
#include "des/Component.h"
#include "des/Simulator.h"
#include "des/Time.h"
#include "des/util/TupleEvent.h"

namespace des {

/*
 * This class is commonly used for scenarios where many adjacent components
 *  are able to trigger a future event but if one is already scheduled only the
 *  single event will be created and executed
*/
template <typename... Types>
class Scheduler {
 public:
  Scheduler(ActiveComponent* _component, EventHandler _handler);
  ~Scheduler();

  void schedule(Time _time, const Types&... _types) const;

 private:
  // TODO(nic): determine if cacheline padding is need here.
  mutable std::atomic<TimeStep> timeStep_;
  mutable TupleEvent<Types...> event_;
};

}  // namespace des

#include "des/util/Scheduler.tcc"

#endif  //  DES_UTIL_SCHEDULER_H_
