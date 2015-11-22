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

#include <cassert>

#include <sstream>

namespace des {

Time::Time()
    : Time(TICK_INV, EPSILON_INV) {}

Time::Time(Tick _tick)
    : Time(_tick, 0) {}

Time::Time(Tick _tick, Epsilon _epsilon)
    : tick(_tick), epsilon(_epsilon) {}

Time::Time(const Time& _o)
    : tick(_o.tick), epsilon(_o.epsilon) {}

Time& Time::operator=(const Time& _o) {
  tick = _o.tick;
  epsilon = _o.epsilon;
  return *this;
}

bool Time::operator==(const Time& _o) const {
  return tick == _o.tick && epsilon == _o.epsilon;
}

bool Time::operator!=(const Time& _o) const {
  return tick != _o.tick || epsilon != _o.epsilon;
}

bool Time::operator<(const Time& _o) const {
  return (tick < _o.tick) || (tick == _o.tick && epsilon < _o.epsilon);
}

bool Time::operator>(const Time& _o) const {
  return (tick > _o.tick) || (tick == _o.tick && epsilon > _o.epsilon);
}

bool Time::operator<=(const Time& _o) const {
  return (tick < _o.tick) || (tick == _o.tick && epsilon < _o.epsilon) ||
      (tick == _o.tick && epsilon == _o.epsilon);
}

bool Time::operator>=(const Time& _o) const {
  return (tick > _o.tick) || (tick == _o.tick && epsilon > _o.epsilon) ||
      (tick == _o.tick && epsilon == _o.epsilon);
}

s32 Time::compare(const Time& _o) const {
  if (tick == _o.tick) {
    if (epsilon == _o.epsilon) {
      return 0;
    } else if (epsilon > _o.epsilon) {
      return 1;
    } else {
      return -1;
    }
  } else if (tick > _o.tick) {
    return 1;
  } else {
    return -1;
  }
}

// NOLINTNEXTLINE(build/include_what_you_use)
Time Time::min(const Time& _a, const Time& _b) {
  return (_a < _b) ? _a : _b;
}

// NOLINTNEXTLINE(build/include_what_you_use)
Time Time::max(const Time& _a, const Time& _b) {
  return (_a > _b) ? _a : _b;
}

Tick Time::operator+(const Time& _o) const {
  Tick ntick = tick + _o.tick;
  assert(ntick >= tick);  // detect overflow
  return ntick;
}

Tick Time::operator+(Tick _tick) const {
  Tick ntick = tick + _tick;
  assert(ntick >= tick);  // detect overflow
  return ntick;
}

Tick Time::operator-(const Time& _o) const {
  Tick ntick = tick - _o.tick;
  assert(ntick <= tick);  // detect underflow
  return ntick;
}

Tick Time::operator-(Tick _tick) const {
  Tick ntick = tick - _tick;
  assert(ntick <= tick);  // detect underflow
  return ntick;
}

std::string Time::toString() const {
  std::stringstream ss;
  ss << static_cast<u64>(tick) << ':' << static_cast<u64>(epsilon);
  return ss.str();
}

}  // namespace des
