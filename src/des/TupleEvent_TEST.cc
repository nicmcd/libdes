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
#include "des/TupleEvent.h"

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

TEST(Event, tuple) {
  des::Simulator sim;
  MyComponent component(&sim);

  std::tuple<u32, f64> exp(0xDEAFBEEF, 3.14159265359);

  {
    des::TupleEvent<u32, f64> evt0;
    ASSERT_EQ(evt0.component, nullptr);
    ASSERT_EQ(evt0.handler, nullptr);
    ASSERT_FALSE(evt0.time.valid());
    ASSERT_NE(evt0.tuple, exp);
    ASSERT_EQ(std::get<0>(evt0.tuple), 0u);
    ASSERT_EQ(evt0.get<0>(), 0u);
    ASSERT_EQ(std::get<1>(evt0.tuple), 0.0);
    ASSERT_EQ(evt0.get<1>(), 0.0);
  }
  {
    des::TupleEvent<u32, f64> evt1(0xDEAFBEEF, 3.14159265359);
    ASSERT_EQ(evt1.component, nullptr);
    ASSERT_EQ(evt1.handler, nullptr);
    ASSERT_FALSE(evt1.time.valid());
    ASSERT_EQ(evt1.tuple, exp);
    ASSERT_EQ(std::get<0>(evt1.tuple), 0xDEAFBEEF);
    ASSERT_EQ(evt1.get<0>(), 0xDEAFBEEF);
    ASSERT_EQ(std::get<1>(evt1.tuple), 3.14159265359);
    ASSERT_EQ(evt1.get<1>(), 3.14159265359);
  }
  {
    des::TupleEvent<u32, f64> evt2(
        &component, makeHandler(MyComponent, ignoreEvent));
    ASSERT_EQ(evt2.component, &component);
    ASSERT_EQ(evt2.handler, static_cast<des::EventHandler>(
        &MyComponent::ignoreEvent));
    ASSERT_FALSE(evt2.time.valid());
    ASSERT_NE(evt2.tuple, exp);
    ASSERT_EQ(std::get<0>(evt2.tuple), 0u);
    ASSERT_EQ(evt2.get<0>(), 0u);
    ASSERT_EQ(std::get<1>(evt2.tuple), 0.0);
    ASSERT_EQ(evt2.get<1>(), 0.0);
  }
  {
    des::TupleEvent<u32, f64> evt3(
        &component, makeHandler(MyComponent, ignoreEvent),
        0xDEAFBEEF, 3.14159265359);
    ASSERT_EQ(evt3.component, &component);
    ASSERT_EQ(evt3.handler, static_cast<des::EventHandler>(
        &MyComponent::ignoreEvent));
    ASSERT_FALSE(evt3.time.valid());
    ASSERT_EQ(evt3.tuple, exp);
    ASSERT_EQ(std::get<0>(evt3.tuple), 0xDEAFBEEF);
    ASSERT_EQ(evt3.get<0>(), 0xDEAFBEEF);
    ASSERT_EQ(std::get<1>(evt3.tuple), 3.14159265359);
    ASSERT_EQ(evt3.get<1>(), 3.14159265359);
  }
  {
    des::TupleEvent<u32, f64> evt4(
        &component, makeHandler(MyComponent, ignoreEvent),
        des::Time(123456789, 9));
    ASSERT_EQ(evt4.component, &component);
    ASSERT_EQ(evt4.handler, static_cast<des::EventHandler>(
        &MyComponent::ignoreEvent));
    ASSERT_TRUE(evt4.time == des::Time(123456789, 9));
    ASSERT_NE(evt4.tuple, exp);
    ASSERT_EQ(std::get<0>(evt4.tuple), 0u);
    ASSERT_EQ(evt4.get<0>(), 0u);
    ASSERT_EQ(std::get<1>(evt4.tuple), 0.0);
    ASSERT_EQ(evt4.get<1>(), 0.0);
  }
  {
    des::TupleEvent<u32, f64> evt5(
        &component, makeHandler(MyComponent, ignoreEvent),
        des::Time(123456789, 9), 0xDEAFBEEF, 3.14159265359);
    ASSERT_EQ(evt5.component, &component);
    ASSERT_EQ(evt5.handler, static_cast<des::EventHandler>(
        &MyComponent::ignoreEvent));
    ASSERT_TRUE(evt5.time == des::Time(123456789, 9));
    ASSERT_EQ(evt5.tuple, exp);
    ASSERT_EQ(std::get<0>(evt5.tuple), 0xDEAFBEEF);
    ASSERT_EQ(evt5.get<0>(), 0xDEAFBEEF);
    ASSERT_EQ(std::get<1>(evt5.tuple), 3.14159265359);
    ASSERT_EQ(evt5.get<1>(), 3.14159265359);
  }
}

TEST(Event, empty) {
  des::Simulator sim;
  MyComponent component(&sim);

  std::tuple<> exp;

  {
    des::TupleEvent<> evt0;
    ASSERT_EQ(evt0.component, nullptr);
    ASSERT_EQ(evt0.handler, nullptr);
    ASSERT_FALSE(evt0.time.valid());
    ASSERT_EQ(evt0.tuple, exp);
  }
  {
    des::TupleEvent<> evt2(
        &component, makeHandler(MyComponent, ignoreEvent));
    ASSERT_EQ(evt2.component, &component);
    ASSERT_EQ(evt2.handler, static_cast<des::EventHandler>(
        &MyComponent::ignoreEvent));
    ASSERT_FALSE(evt2.time.valid());
    ASSERT_EQ(evt2.tuple, exp);
  }
  {
    des::TupleEvent<> evt4(
        &component, makeHandler(MyComponent, ignoreEvent),
        des::Time(123456789, 9));
    ASSERT_EQ(evt4.component, &component);
    ASSERT_EQ(evt4.handler, static_cast<des::EventHandler>(
        &MyComponent::ignoreEvent));
    ASSERT_TRUE(evt4.time == des::Time(123456789, 9));
    ASSERT_EQ(evt4.tuple, exp);
  }
}
