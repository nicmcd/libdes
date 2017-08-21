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
#include "des/ItemEvent.h"

#include <gtest/gtest.h>

#include "des/ActiveComponent.h"
#include "des/Component.h"
#include "des/Simulator.h"
#include "des/Time.h"

namespace {
class MyComponent : public des::ActiveComponent {
 public:
  explicit MyComponent(des::Simulator* _sim)
      : des::ActiveComponent(_sim, "component") {}
  void ignoreEvent(des::Event*) {}
};
}  // namespace

TEST(Event, item) {
  des::Simulator sim;
  MyComponent component(&sim);
  des::ItemEvent<u32> evt(
      &component, makeHandler(MyComponent, ignoreEvent),
      des::Time(123456789, 9), 0xDEAFBEEF);
  ASSERT_EQ(evt.component, &component);
  ASSERT_EQ(evt.handler, static_cast<des::EventHandler>(
      &MyComponent::ignoreEvent));
  ASSERT_TRUE(evt.time == des::Time(123456789, 9));
  ASSERT_EQ(evt.item, 0xDEAFBEEF);
}
