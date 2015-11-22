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
#ifndef DES_TIME_H_
#define DES_TIME_H_

#include <prim/prim.h>

#include <string>

namespace des {

// these define the size and bounds of time fields
typedef u64 Tick;
static const Tick TICK_INV = U64_MAX;
typedef u8 Epsilon;
static const Epsilon EPSILON_INV = U8_MAX;

// this class handles the bulk of time operations
class Time {
 public:
  // initializes time as invalid
  Time();

  // initializes tick as specified, epsilon to zero
  explicit Time(Tick _tick);

  // initializes tick as specified, epsilon as specified
  Time(Tick _tick, Epsilon _epsilon);

  // copy constructor
  Time(const Time& _o);

  // assignment operator
  Time& operator=(const Time& o);

  // comparison operators
  bool operator==(const Time& _o) const;
  bool operator!=(const Time& _o) const;
  bool operator<(const Time& _o) const;
  bool operator>(const Time& _o) const;
  bool operator<=(const Time& _o) const;
  bool operator>=(const Time& _o) const;
  s32 compare(const Time& _o) const;

  // helper min/max functions
  static Time min(const Time& _a, const Time& _b);  // NOLINT
  static Time max(const Time& _a, const Time& _b);  // NOLINT

  // these operators perform time arithmetic (on tick)
  Tick operator+(const Time& _o) const;
  Tick operator+(Tick _tick) const;
  Tick operator-(const Time& _o) const;
  Tick operator-(Tick _tick) const;

  // misc helper functions
  std::string toString() const;
  bool valid() const;

  Tick tick;
  Epsilon epsilon;
};

}  // namespace des

#endif  // DES_TIME_H_
