/*
 * Copyright (c) 2012-2016, Nic McDonald
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
 * - Neither the name of prim nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
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

#include <gtest/gtest.h>

#include "des/Model.h"
#include "des/Simulator.h"
#include "des/Time.h"

class MyModel : public des::Model {
 public:
  explicit MyModel(des::Simulator* _sim)
      : des::Model(_sim, "model", nullptr) {}
  void ignoreEvent(des::Event*) {}
};

TEST(Event, constructor1) {
  des::Simulator sim;
  MyModel model(&sim);
  des::Event evt;
  ASSERT_EQ(evt.model, nullptr);
  ASSERT_EQ(evt.handler, nullptr);
  ASSERT_TRUE(evt.time ==  des::Time());
}

TEST(Event, constructor2) {
  des::Simulator sim;
  MyModel model(&sim);
  des::Event evt(&model,
                 static_cast<des::EventHandler>(&MyModel::ignoreEvent));
  ASSERT_EQ(evt.model, &model);
  ASSERT_EQ(evt.handler, static_cast<des::EventHandler>(&MyModel::ignoreEvent));
  ASSERT_TRUE(evt.time == des::Time());
}

TEST(Event, constructor3) {
  des::Simulator sim;
  MyModel model(&sim);
  des::Time etime(123456789, 9);
  des::Event evt(&model,
                 static_cast<des::EventHandler>(&MyModel::ignoreEvent),
                 etime);
  ASSERT_EQ(evt.model, &model);
  ASSERT_EQ(evt.handler, static_cast<des::EventHandler>(&MyModel::ignoreEvent));
  ASSERT_TRUE(evt.time == etime);
}

TEST(Event, constructor3b) {
  des::Simulator sim;
  MyModel model(&sim);
  des::Event evt(&model,
                 static_cast<des::EventHandler>(&MyModel::ignoreEvent),
                 des::Time(123456789, 9));
  ASSERT_EQ(evt.model, &model);
  ASSERT_EQ(evt.handler, static_cast<des::EventHandler>(&MyModel::ignoreEvent));
  ASSERT_TRUE(evt.time == des::Time(123456789, 9));
}

TEST(Event, eventCompare) {
  des::EventComparator comp;

  des::Event e1;
  des::Event e2;

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

TEST(Event, item) {
  des::Simulator sim;
  MyModel model(&sim);
  des::ItemEvent<u32> evt(
      &model, static_cast<des::EventHandler>(&MyModel::ignoreEvent),
      des::Time(123456789, 9), 0xDEAFBEEF);
  ASSERT_EQ(evt.model, &model);
  ASSERT_EQ(evt.handler, static_cast<des::EventHandler>(&MyModel::ignoreEvent));
  ASSERT_TRUE(evt.time == des::Time(123456789, 9));
  ASSERT_EQ(evt.item, 0xDEAFBEEF);
}
