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
#include "des/Event.h"

#include "des/ActiveComponent.h"
#include "des/Component.h"
#include "des/Simulator.h"
#include "des/Time.h"
#include "gtest/gtest.h"

namespace {
class MyComponent : public des::ActiveComponent {
 public:
  explicit MyComponent(des::Simulator* _sim)
      : des::ActiveComponent(_sim, "component") {}
  void ignoreEvent() {}
};
}  // namespace

TEST(Event, constructor1) {
  des::Simulator sim(1);
  MyComponent component(&sim);
  des::Event evt(&component);
  ASSERT_EQ(evt.handler, nullptr);
  ASSERT_TRUE(evt.time == des::Time());
  ASSERT_FALSE(evt.skip);
  ASSERT_FALSE(evt.clean);
  ASSERT_EQ(evt.executer(), component.executer());
  ASSERT_FALSE(evt.enqueued());
}

TEST(Event, constructor2) {
  des::Simulator sim(1);
  MyComponent component(&sim);
  des::Event evt(&component, std::bind(&MyComponent::ignoreEvent, &component));
  ASSERT_NE(evt.handler, nullptr);
  ASSERT_TRUE(evt.time == des::Time());
  ASSERT_FALSE(evt.skip);
  ASSERT_FALSE(evt.clean);
  ASSERT_EQ(evt.executer(), component.executer());
  ASSERT_FALSE(evt.enqueued());
}

TEST(Event, constructor3) {
  des::Simulator sim(1);
  MyComponent component(&sim);
  des::Time etime(123456789, 9);
  des::Event evt(&component, std::bind(&MyComponent::ignoreEvent, &component),
                 etime);
  ASSERT_NE(evt.handler, nullptr);
  ASSERT_TRUE(evt.time == etime);
  ASSERT_FALSE(evt.skip);
  ASSERT_FALSE(evt.clean);
  ASSERT_EQ(evt.executer(), component.executer());
  ASSERT_FALSE(evt.enqueued());
}

TEST(Event, constructor3b) {
  des::Simulator sim(1);
  MyComponent component(&sim);
  des::Event evt(&component, std::bind(&MyComponent::ignoreEvent, &component),
                 des::Time(123456789, 9));
  ASSERT_NE(evt.handler, nullptr);
  ASSERT_TRUE(evt.time == des::Time(123456789, 9));
  ASSERT_FALSE(evt.skip);
  ASSERT_FALSE(evt.clean);
  ASSERT_EQ(evt.executer(), component.executer());
  ASSERT_FALSE(evt.enqueued());
}

TEST(Event, constructor4) {
  des::Simulator sim(1);
  MyComponent component(&sim);
  des::Time etime(123456789, 9);
  des::Event evt(&component, std::bind(&MyComponent::ignoreEvent, &component),
                 etime, true);
  ASSERT_NE(evt.handler, nullptr);
  ASSERT_TRUE(evt.time == etime);
  ASSERT_FALSE(evt.skip);
  ASSERT_TRUE(evt.clean);
  ASSERT_EQ(evt.executer(), component.executer());
  ASSERT_FALSE(evt.enqueued());
}

TEST(Event, move) {
  des::Simulator sim(1);
  MyComponent component(&sim);
  des::Event evt(&component);
  ASSERT_EQ(evt.handler, nullptr);
  ASSERT_TRUE(evt.time == des::Time());
  ASSERT_FALSE(evt.skip);
  ASSERT_FALSE(evt.clean);
  ASSERT_EQ(evt.executer(), component.executer());
  ASSERT_FALSE(evt.enqueued());

  des::Time etime(123456789, 9);
  evt = des::Event(&component, std::bind(&MyComponent::ignoreEvent, &component),
                   etime, true);
  ASSERT_NE(evt.handler, nullptr);
  ASSERT_TRUE(evt.time == etime);
  ASSERT_FALSE(evt.skip);
  ASSERT_TRUE(evt.clean);
  ASSERT_EQ(evt.executer(), component.executer());
  ASSERT_FALSE(evt.enqueued());
}

TEST(Event, eventCompare) {
  des::EventComparator comp;

  des::Simulator sim(1);
  MyComponent component(&sim);
  des::Event e1(&component);
  des::Event e2(&component);

  e1.time = des::Time(100, 0);
  e2.time = des::Time(99, 1);
  ASSERT_TRUE(comp(&e1, &e2));

  e1.time = des::Time(100, 1);
  e2.time = des::Time(100, 0);
  ASSERT_TRUE(comp(&e1, &e2));

  e1.time = des::Time(100, 0);
  e2.time = des::Time(100, 0);
  ASSERT_FALSE(comp(&e1, &e2));

  e1.time = des::Time(100, 0);
  e2.time = des::Time(101, 0);
  ASSERT_FALSE(comp(&e1, &e2));

  e1.time = des::Time(100, 0);
  e2.time = des::Time(100, 1);
  ASSERT_FALSE(comp(&e1, &e2));
}
