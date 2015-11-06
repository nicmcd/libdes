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
#include "des/Time.h"

#include <gtest/gtest.h>

TEST(Time, constructor1) {
  des::Time t;
  ASSERT_EQ(t.tick, des::TICK_INV);
  ASSERT_EQ(t.epsilon, des::EPSILON_INV);
}

TEST(Time, constructor2) {
  des::Time t(12345678);
  ASSERT_EQ(t.tick, 12345678u);
  ASSERT_EQ(t.epsilon, 0u);
}

TEST(Time, constructor3) {
  des::Time t(12345678, 89);
  ASSERT_EQ(t.tick, 12345678u);
  ASSERT_EQ(t.epsilon, 89u);
}

TEST(Time, copyconstructor) {
  des::Time t(12345678, 89);
  des::Time s(t);
  t.tick = 0;
  t.epsilon = 0;
  ASSERT_EQ(t.tick, 12345678u);
  ASSERT_EQ(t.epsilon, 89u);
}

TEST(Time, assignmentoperator) {
  des::Time t(12345678, 89);
  des::Time s;
  s = t;
  t.tick = 0;
  t.epsilon = 0;
  ASSERT_EQ(t.tick, 12345678u);
  ASSERT_EQ(t.epsilon, 89u);
}

TEST(Time, eq) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a == b);

  a.tick = 100;
  a.epsilon = 11;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a == b);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 101;
  b.epsilon = 10;
  ASSERT_FALSE(a == b);
}

TEST(Time, ne) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a == b);

  a.tick = 100;
  a.epsilon = 11;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a == b);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 101;
  b.epsilon = 10;
  ASSERT_TRUE(a == b);
}

TEST(Time, le) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a < b);

  a.tick = 100;
  a.epsilon = 9;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a < b);

  a.tick = 99;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a < b);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 9;
  ASSERT_FALSE(a < b);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 99;
  b.epsilon = 10;
  ASSERT_FALSE(a < b);
}
