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
  ASSERT_EQ(t.tick(), des::TICK_INV);
  ASSERT_EQ(t.epsilon(), des::EPSILON_INV);
}

TEST(Time, constructor2) {
  des::Time t(12345678);
  ASSERT_EQ(t.tick(), 12345678u);
  ASSERT_EQ(t.epsilon(), 0u);
}

TEST(Time, constructor2_implicit) {
  des::Time s;
  ASSERT_EQ(s.tick(), des::TICK_INV);
  ASSERT_EQ(s.epsilon(), des::EPSILON_INV);

  s = 123;
  ASSERT_EQ(s.tick(), 123u);
  ASSERT_EQ(s.epsilon(), 0u);

  s.setEpsilon(13);
  s = 199;
  ASSERT_EQ(s.tick(), 199u);
  ASSERT_EQ(s.epsilon(), 0u);

  des::Time t = (des::Tick)5;
  ASSERT_EQ(t.tick(), 5u);
  ASSERT_EQ(t.epsilon(), 0u);
}

TEST(Time, constructor3) {
  des::Time t(12345678, 13);
  ASSERT_EQ(t.tick(), 12345678u);
  ASSERT_EQ(t.epsilon(), 13u);
}

TEST(Time, copyconstructor) {
  des::Time t(12345678, 13);
  des::Time s(t);
  t.setTick(0);
  t.setEpsilon(0);
  ASSERT_EQ(t.tick(), 0u);
  ASSERT_EQ(t.epsilon(), 0u);
  ASSERT_EQ(s.tick(), 12345678u);
  ASSERT_EQ(s.epsilon(), 13u);
}

TEST(Time, assignmentoperator1) {
  des::Time t(12345678, 13);
  des::Time s;
  s = t;
  t.setTick(0);
  t.setEpsilon(0);
  ASSERT_EQ(t.tick(), 0u);
  ASSERT_EQ(t.epsilon(), 0u);
  ASSERT_EQ(s.tick(), 12345678u);
  ASSERT_EQ(s.epsilon(), 13u);
}

TEST(Time, create) {
  des::Time t = des::Time::create(1lu << des::EPSILON_BITS | 1lu);
  ASSERT_EQ(t.tick(), 1u);
  ASSERT_EQ(t.epsilon(), 1u);

  t = des::Time::create(~0);
  ASSERT_EQ(t.tick(), des::TICK_INV);
  ASSERT_EQ(t.epsilon(), des::EPSILON_INV);

  t = des::Time::create(0);
  ASSERT_EQ(t.tick(), 0u);
  ASSERT_EQ(t.epsilon(), 0u);
}


TEST(Time, eq) {
  des::Time a;
  des::Time b;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_TRUE(a == b);
  ASSERT_TRUE(a.operator==(b));

  a.setTick(100);
  a.setEpsilon(11);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_FALSE(a == b);
  ASSERT_FALSE(a.operator==(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(101);
  b.setEpsilon(10);
  ASSERT_FALSE(a == b);
  ASSERT_FALSE(a.operator==(b));
}

TEST(Time, ne) {
  des::Time a;
  des::Time b;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_FALSE(a != b);
  ASSERT_FALSE(a.operator!=(b));

  a.setTick(100);
  a.setEpsilon(11);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_TRUE(a != b);
  ASSERT_TRUE(a.operator!=(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(101);
  b.setEpsilon(10);
  ASSERT_TRUE(a != b);
  ASSERT_TRUE(a.operator!=(b));
}

TEST(Time, lt) {
  des::Time a;
  des::Time b;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_FALSE(a < b);
  ASSERT_FALSE(a.operator<(b));

  a.setTick(100);
  a.setEpsilon(9);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_TRUE(a < b);
  ASSERT_TRUE(a.operator<(b));

  a.setTick(99);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_TRUE(a < b);
  ASSERT_TRUE(a.operator<(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(9);
  ASSERT_FALSE(a < b);
  ASSERT_FALSE(a.operator<(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(99);
  b.setEpsilon(10);
  ASSERT_FALSE(a < b);
  ASSERT_FALSE(a.operator<(b));
}

TEST(Time, gt) {
  des::Time a;
  des::Time b;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_FALSE(a > b);
  ASSERT_FALSE(a.operator>(b));

  a.setTick(100);
  a.setEpsilon(9);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_FALSE(a > b);
  ASSERT_FALSE(a.operator>(b));

  a.setTick(99);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_FALSE(a > b);
  ASSERT_FALSE(a.operator>(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(9);
  ASSERT_TRUE(a > b);
  ASSERT_TRUE(a.operator>(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(99);
  b.setEpsilon(10);
  ASSERT_TRUE(a > b);
  ASSERT_TRUE(a.operator>(b));
}

TEST(Time, le) {
  des::Time a;
  des::Time b;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_TRUE(a <= b);
  ASSERT_TRUE(a.operator<=(b));

  a.setTick(100);
  a.setEpsilon(9);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_TRUE(a <= b);
  ASSERT_TRUE(a.operator<=(b));

  a.setTick(99);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_TRUE(a <= b);
  ASSERT_TRUE(a.operator<=(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(9);
  ASSERT_FALSE(a <= b);
  ASSERT_FALSE(a.operator<=(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(99);
  b.setEpsilon(10);
  ASSERT_FALSE(a <= b);
  ASSERT_FALSE(a.operator<=(b));
}

TEST(Time, ge) {
  des::Time a;
  des::Time b;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_TRUE(a >= b);
  ASSERT_TRUE(a.operator>=(b));

  a.setTick(100);
  a.setEpsilon(9);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_FALSE(a >= b);
  ASSERT_FALSE(a.operator>=(b));

  a.setTick(99);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_FALSE(a >= b);
  ASSERT_FALSE(a.operator>=(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(9);
  ASSERT_TRUE(a >= b);
  ASSERT_TRUE(a.operator>=(b));

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(99);
  b.setEpsilon(10);
  ASSERT_TRUE(a >= b);
  ASSERT_TRUE(a.operator>=(b));
}

TEST(Time, compare) {
  des::Time a;
  des::Time b;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_EQ(a.compare(b), 0);
  ASSERT_EQ(b.compare(a), 0);

  a.setTick(100);
  a.setEpsilon(9);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_EQ(a.compare(b), -1);
  ASSERT_EQ(b.compare(a), 1);

  a.setTick(99);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  ASSERT_EQ(a.compare(b), -1);
  ASSERT_EQ(b.compare(a), 1);

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(9);
  ASSERT_EQ(a.compare(b), 1);
  ASSERT_EQ(b.compare(a), -1);

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(99);
  b.setEpsilon(10);
  ASSERT_EQ(a.compare(b), 1);
  ASSERT_EQ(b.compare(a), -1);
}

TEST(Time, min) {
  des::Time a;
  des::Time b;
  des::Time c;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick(), 100u);
  ASSERT_EQ(c.epsilon(), 10u);

  a.setTick(100);
  a.setEpsilon(9);
  b.setTick(100);
  b.setEpsilon(10);
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick(), 100u);
  ASSERT_EQ(c.epsilon(), 9u);

  a.setTick(99);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick(), 99u);
  ASSERT_EQ(c.epsilon(), 10u);

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(9);
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick(), 100u);
  ASSERT_EQ(c.epsilon(), 9u);

  a.setTick(100);
  a.setEpsilon(2);
  b.setTick(99);
  b.setEpsilon(10);
  c = des::Time::min(a, b);
  ASSERT_EQ(c.tick(), 99u);
  ASSERT_EQ(c.epsilon(), 10u);
}

TEST(Time, max) {
  des::Time a;
  des::Time b;
  des::Time c;

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick(), 100u);
  ASSERT_EQ(c.epsilon(), 10u);

  a.setTick(100);
  a.setEpsilon(9);
  b.setTick(100);
  b.setEpsilon(10);
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick(), 100u);
  ASSERT_EQ(c.epsilon(), 10u);

  a.setTick(99);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(10);
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick(), 100u);
  ASSERT_EQ(c.epsilon(), 10u);

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(100);
  b.setEpsilon(9);
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick(), 100u);
  ASSERT_EQ(c.epsilon(), 10u);

  a.setTick(100);
  a.setEpsilon(10);
  b.setTick(99);
  b.setEpsilon(11);
  c = des::Time::max(a, b);
  ASSERT_EQ(c.tick(), 100u);
  ASSERT_EQ(c.epsilon(), 10u);
}

TEST(Time, plus) {
  des::Time a;
  des::Time b;
  des::Time c;

  a.setTick(50);
  a.setEpsilon(6);
  b.setTick(15);
  b.setEpsilon(7);
  c = a + b;
  ASSERT_EQ(c.tick(), 65u);
  ASSERT_EQ(c.epsilon(), 0u);

  a.setTick(5);
  a.setEpsilon(1);
  b.setTick(89);
  b.setEpsilon(2);
  c = a + b;
  ASSERT_EQ(c.tick(), 94u);
  ASSERT_EQ(c.epsilon(), 0u);

  a.setTick(5);
  a.setEpsilon(1);
  c = a + 10;
  ASSERT_EQ(c.tick(), 15u);
  ASSERT_EQ(c.epsilon(), 0u);
}

TEST(Time, minus) {
  des::Time a;
  des::Time b;
  des::Time c;

  a.setTick(50);
  a.setEpsilon(6);
  b.setTick(15);
  b.setEpsilon(7);
  c = a - b;
  ASSERT_EQ(c.tick(), 35u);
  ASSERT_EQ(c.epsilon(), 0u);

  a.setTick(105);
  a.setEpsilon(1);
  b.setTick(89);
  b.setEpsilon(2);
  c = a - b;
  ASSERT_EQ(c.tick(), 16u);
  ASSERT_EQ(c.epsilon(), 0u);

  a.setTick(34);
  a.setEpsilon(1);
  c = a - 10;
  ASSERT_EQ(c.tick(), 24u);
  ASSERT_EQ(c.epsilon(), 0u);
}

TEST(Time, plusEqualsTime) {
  des::Time a(100, 10);
  des::Time b(0, 5);
  des::Time c(100, 13);

  a += b;
  ASSERT_EQ(a.tick(), 100u);
  ASSERT_EQ(a.epsilon(), 0u);

  a += c;
  ASSERT_EQ(a.tick(), 200u);
  ASSERT_EQ(a.epsilon(), 0u);
}

TEST(Time, plusEqualsTick) {
  des::Time a(100, 10);
  des::Tick b = 0;
  des::Tick c = 100;

  a += b;
  ASSERT_EQ(a.tick(), 100u);
  ASSERT_EQ(a.epsilon(), 0u);

  a += c;
  ASSERT_EQ(a.tick(), 200u);
  ASSERT_EQ(a.epsilon(), 0u);
}

TEST(Time, minusEqualsTime) {
  des::Time a(100, 10);
  des::Time b(0, 5);
  des::Time c(100, 13);

  a -= b;
  ASSERT_EQ(a.tick(), 100u);
  ASSERT_EQ(a.epsilon(), 0u);

  a -= c;
  ASSERT_EQ(a.tick(), 0u);
  ASSERT_EQ(a.epsilon(), 0u);
}

TEST(Time, minusEqualsTick) {
  des::Time a(100, 10);
  des::Tick b = 0;
  des::Tick c = 100;

  a -= b;
  ASSERT_EQ(a.tick(), 100u);
  ASSERT_EQ(a.epsilon(), 0u);

  a -= c;
  ASSERT_EQ(a.tick(), 0u);
  ASSERT_EQ(a.epsilon(), 0u);
}

TEST(Time, nextEpsilon) {
  des::Time a;
  a.setTick(34);
  a.setEpsilon(3);
  des::Time b = a.nextEpsilon();
  ASSERT_EQ(a.tick(), 34u);
  ASSERT_EQ(a.epsilon(), 3u);
  ASSERT_EQ(b.tick(), 34u);
  ASSERT_EQ(b.epsilon(), 4u);
}

TEST(Time, incrEpsilon) {
  des::Time a;
  a.setTick(34);
  a.setEpsilon(3);
  ASSERT_EQ(a.tick(), 34u);
  ASSERT_EQ(a.epsilon(), 3u);
  a.incrEpsilon();
  ASSERT_EQ(a.tick(), 34u);
  ASSERT_EQ(a.epsilon(), 4u);
}

TEST(Time, toString) {
  ASSERT_EQ(des::Time(50, 13).toString(), "50:13");
  ASSERT_EQ(des::Time(43).toString(), "43:0");
  ASSERT_EQ(des::Time().toString(), std::to_string(des::TICK_INV) +
            ':' + std::to_string(des::EPSILON_INV));
  ASSERT_EQ(des::Time(199, 2).toString(), "199:2");
}

TEST(Time, valid) {
  des::Time t;
  ASSERT_FALSE(t.valid());
  t.setTick(0);
  ASSERT_FALSE(t.valid());
  t.setEpsilon(0);
  ASSERT_TRUE(t.valid());
  t.setTick(des::TICK_INV);
  ASSERT_FALSE(t.valid());
}

TEST(Time, raw) {
  des::Time a(34, 3);
  ASSERT_EQ(a.raw(), (34lu << des::EPSILON_BITS) + 3);
}
