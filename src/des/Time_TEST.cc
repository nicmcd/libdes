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

TEST(Time, constructor2_implicit) {
  des::Time s;
  ASSERT_EQ(s.tick, des::TICK_INV);
  ASSERT_EQ(s.epsilon, des::EPSILON_INV);

  s = 123;
  ASSERT_EQ(s.tick, 123u);
  ASSERT_EQ(s.epsilon, 0);

  s.epsilon = 230;
  s = 199;
  ASSERT_EQ(s.tick, 199u);
  ASSERT_EQ(s.epsilon, 0);

  des::Time t = (des::Tick)5;
  ASSERT_EQ(t.tick, 5u);
  ASSERT_EQ(t.epsilon, 0);
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
  ASSERT_EQ(t.tick, 0u);
  ASSERT_EQ(t.epsilon, 0u);
  ASSERT_EQ(s.tick, 12345678u);
  ASSERT_EQ(s.epsilon, 89u);
}

TEST(Time, assignmentoperator1) {
  des::Time t(12345678, 89);
  des::Time s;
  s = t;
  t.tick = 0;
  t.epsilon = 0;
  ASSERT_EQ(t.tick, 0u);
  ASSERT_EQ(t.epsilon, 0u);
  ASSERT_EQ(s.tick, 12345678u);
  ASSERT_EQ(s.epsilon, 89u);
}

TEST(Time, eq) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a == b);
  ASSERT_TRUE(a.operator==(b));

  a.tick = 100;
  a.epsilon = 11;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a == b);
  ASSERT_FALSE(a.operator==(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 101;
  b.epsilon = 10;
  ASSERT_FALSE(a == b);
  ASSERT_FALSE(a.operator==(b));
}

TEST(Time, ne) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a != b);
  ASSERT_FALSE(a.operator!=(b));

  a.tick = 100;
  a.epsilon = 11;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a != b);
  ASSERT_TRUE(a.operator!=(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 101;
  b.epsilon = 10;
  ASSERT_TRUE(a != b);
  ASSERT_TRUE(a.operator!=(b));
}

TEST(Time, lt) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a < b);
  ASSERT_FALSE(a.operator<(b));

  a.tick = 100;
  a.epsilon = 9;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a < b);
  ASSERT_TRUE(a.operator<(b));

  a.tick = 99;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a < b);
  ASSERT_TRUE(a.operator<(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 9;
  ASSERT_FALSE(a < b);
  ASSERT_FALSE(a.operator<(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 99;
  b.epsilon = 10;
  ASSERT_FALSE(a < b);
  ASSERT_FALSE(a.operator<(b));
}

TEST(Time, gt) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a > b);
  ASSERT_FALSE(a.operator>(b));

  a.tick = 100;
  a.epsilon = 9;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a > b);
  ASSERT_FALSE(a.operator>(b));

  a.tick = 99;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a > b);
  ASSERT_FALSE(a.operator>(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 9;
  ASSERT_TRUE(a > b);
  ASSERT_TRUE(a.operator>(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 99;
  b.epsilon = 10;
  ASSERT_TRUE(a > b);
  ASSERT_TRUE(a.operator>(b));
}

TEST(Time, le) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a <= b);
  ASSERT_TRUE(a.operator<=(b));

  a.tick = 100;
  a.epsilon = 9;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a <= b);
  ASSERT_TRUE(a.operator<=(b));

  a.tick = 99;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a <= b);
  ASSERT_TRUE(a.operator<=(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 9;
  ASSERT_FALSE(a <= b);
  ASSERT_FALSE(a.operator<=(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 99;
  b.epsilon = 10;
  ASSERT_FALSE(a <= b);
  ASSERT_FALSE(a.operator<=(b));
}

TEST(Time, ge) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_TRUE(a >= b);
  ASSERT_TRUE(a.operator>=(b));

  a.tick = 100;
  a.epsilon = 9;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a >= b);
  ASSERT_FALSE(a.operator>=(b));

  a.tick = 99;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_FALSE(a >= b);
  ASSERT_FALSE(a.operator>=(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 9;
  ASSERT_TRUE(a >= b);
  ASSERT_TRUE(a.operator>=(b));

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 99;
  b.epsilon = 10;
  ASSERT_TRUE(a >= b);
  ASSERT_TRUE(a.operator>=(b));
}

TEST(Time, compare) {
  des::Time a;
  des::Time b;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_EQ(a.compare(b), 0);
  ASSERT_EQ(b.compare(a), 0);

  a.tick = 100;
  a.epsilon = 9;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_EQ(a.compare(b), -1);
  ASSERT_EQ(b.compare(a), 1);

  a.tick = 99;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  ASSERT_EQ(a.compare(b), -1);
  ASSERT_EQ(b.compare(a), 1);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 9;
  ASSERT_EQ(a.compare(b), 1);
  ASSERT_EQ(b.compare(a), -1);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 99;
  b.epsilon = 10;
  ASSERT_EQ(a.compare(b), 1);
  ASSERT_EQ(b.compare(a), -1);
}

TEST(Time, min) {
  des::Time a;
  des::Time b;
  des::Time c;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick, 100u);
  ASSERT_EQ(c.epsilon, 10u);

  a.tick = 100;
  a.epsilon = 9;
  b.tick = 100;
  b.epsilon = 10;
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick, 100u);
  ASSERT_EQ(c.epsilon, 9u);

  a.tick = 99;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick, 99u);
  ASSERT_EQ(c.epsilon, 10u);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 9;
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick, 100u);
  ASSERT_EQ(c.epsilon, 9u);

  a.tick = 100;
  a.epsilon = 2;
  b.tick = 99;
  b.epsilon = 10;
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick, 99u);
  ASSERT_EQ(c.epsilon, 10u);
}

TEST(Time, max) {
  des::Time a;
  des::Time b;
  des::Time c;

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick, 100u);
  ASSERT_EQ(c.epsilon, 10u);

  a.tick = 100;
  a.epsilon = 9;
  b.tick = 100;
  b.epsilon = 10;
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick, 100u);
  ASSERT_EQ(c.epsilon, 10u);

  a.tick = 99;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 10;
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick, 100u);
  ASSERT_EQ(c.epsilon, 10u);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 100;
  b.epsilon = 9;
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick, 100u);
  ASSERT_EQ(c.epsilon, 10u);

  a.tick = 100;
  a.epsilon = 10;
  b.tick = 99;
  b.epsilon = 11;
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick, 100u);
  ASSERT_EQ(c.epsilon, 10u);
}

TEST(Time, plus) {
  des::Time a;
  des::Time b;
  des::Tick c;

  a.tick = 50;
  a.epsilon = 6;
  b.tick = 15;
  b.epsilon = 7;
  c = a + b;
  ASSERT_EQ(c, 65u);

  a.tick = 5;
  a.epsilon = 1;
  b.tick = 89;
  b.epsilon = 2;
  c = a + b;
  ASSERT_EQ(c, 94u);

  a.tick = 5;
  a.epsilon = 1;
  c = a + 10;
  ASSERT_EQ(c, 15u);
}

TEST(Time, minus) {
  des::Time a;
  des::Time b;
  des::Tick c;

  a.tick = 50;
  a.epsilon = 6;
  b.tick = 15;
  b.epsilon = 7;
  c = a - b;
  ASSERT_EQ(c, 35u);

  a.tick = 105;
  a.epsilon = 1;
  b.tick = 89;
  b.epsilon = 2;
  c = a - b;
  ASSERT_EQ(c, 16u);

  a.tick = 34;
  a.epsilon = 1;
  c = a - 10;
  ASSERT_EQ(c, 24u);
}

TEST(Time, plusEqualsTime) {
  des::Time a(100, 10);
  des::Time b(0, 20);
  des::Time c(100, 30);

  a += b;
  ASSERT_EQ(a.tick, 100u);
  ASSERT_EQ(a.epsilon, 10u);

  a += c;
  ASSERT_EQ(a.tick, 200u);
  ASSERT_EQ(a.epsilon, 0u);
}

TEST(Time, plusEqualsTick) {
  des::Time a(100, 10);
  des::Tick b = 0;
  des::Tick c = 100;

  a += b;
  ASSERT_EQ(a.tick, 100u);
  ASSERT_EQ(a.epsilon, 10u);

  a += c;
  ASSERT_EQ(a.tick, 200u);
  ASSERT_EQ(a.epsilon, 0u);
}

TEST(Time, minusEqualsTime) {
  des::Time a(100, 10);
  des::Time b(0, 20);
  des::Time c(100, 30);

  a -= b;
  ASSERT_EQ(a.tick, 100u);
  ASSERT_EQ(a.epsilon, 10u);

  a -= c;
  ASSERT_EQ(a.tick, 0u);
  ASSERT_EQ(a.epsilon, 0u);
}

TEST(Time, minusEqualsTick) {
  des::Time a(100, 10);
  des::Tick b = 0;
  des::Tick c = 100;

  a -= b;
  ASSERT_EQ(a.tick, 100u);
  ASSERT_EQ(a.epsilon, 10u);

  a -= c;
  ASSERT_EQ(a.tick, 0u);
  ASSERT_EQ(a.epsilon, 0u);
}

TEST(Time, plusEps) {
  des::Time a;

  a.tick = 34;
  a.epsilon = 1;
  des::Time b = a.plusEps();
  ASSERT_EQ(b, des::Time(34, 2));
  ASSERT_EQ(a, des::Time(34, 1));
}

TEST(Time, toString) {
  ASSERT_EQ(des::Time(50, 67).toString(), "50:67");
  ASSERT_EQ(des::Time(43).toString(), "43:0");
  ASSERT_EQ(des::Time().toString(), std::to_string(des::TICK_INV) +
            ':' + std::to_string(des::EPSILON_INV));
  ASSERT_EQ(des::Time(199, 201).toString(), "199:201");
}

TEST(Time, valid) {
  des::Time t;
  ASSERT_FALSE(t.valid());
  t.tick = 0;
  ASSERT_FALSE(t.valid());
  t.epsilon = 0;
  ASSERT_TRUE(t.valid());
  t.tick = des::TICK_INV;
  ASSERT_FALSE(t.valid());
}
