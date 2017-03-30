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

#include <cassert>

#include <sstream>

namespace des {

Time::Time()
    : timeStep_(~0) {}

Time::Time(Tick _tick)
    : Time(_tick, 0) {}

Time::Time(Tick _tick, Epsilon _epsilon) {
  assert(_tick <= TICK_INV);
  assert(_epsilon <= EPSILON_INV);
  timeStep_ = _tick << EPSILON_BITS;
  timeStep_ |= _epsilon;
}

Time::Time(const Time& _o)
    : timeStep_(_o.timeStep_) {}

Time& Time::operator=(const Time& _o) {
  timeStep_ = _o.timeStep_;
  return *this;
}

Time Time::create(TimeStep _timeStep) {
  Time time;
  time.timeStep_ = _timeStep;
  return time;
}

Tick Time::tick() const {
  return (timeStep_ & ~EPSILON_INV) >> EPSILON_BITS;
}

Epsilon Time::epsilon() const {
  return timeStep_ & EPSILON_INV;
}

void Time::setTick(Tick _tick) {
  assert(_tick <= TICK_INV);
  timeStep_ = (_tick << EPSILON_BITS) | (timeStep_ & EPSILON_INV);
}

void Time::setEpsilon(Epsilon _epsilon) {
  assert(_epsilon <= EPSILON_INV);
  timeStep_ = _epsilon | (timeStep_ & ~EPSILON_INV);
}

bool Time::operator==(const Time& _o) const {
  return timeStep_ == _o.timeStep_;
}

bool Time::operator!=(const Time& _o) const {
  return timeStep_ != _o.timeStep_;
}

bool Time::operator<(const Time& _o) const {
  return timeStep_ < _o.timeStep_;
}

bool Time::operator>(const Time& _o) const {
  return timeStep_ > _o.timeStep_;
}

bool Time::operator<=(const Time& _o) const {
  return timeStep_ <= _o.timeStep_;
}

bool Time::operator>=(const Time& _o) const {
  return timeStep_ >= _o.timeStep_;
}

s32 Time::compare(const Time& _o) const {
  if (timeStep_ == _o.timeStep_) {
    return 0;
  } else if (timeStep_ > _o.timeStep_) {
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

Time Time::operator+(const Time& _o) const {
  Tick ntick = tick() + _o.tick();
  assert(ntick >= tick());  // detect overflow
  return create(ntick << EPSILON_BITS);
}

Time Time::operator+(Tick _tick) const {
  assert(_tick <= TICK_INV);
  Tick ntick = tick() + _tick;
  assert(ntick >= tick());  // detect overflow
  return create(ntick << EPSILON_BITS);
}

Time Time::operator-(const Time& _o) const {
  Tick ntick = tick() - _o.tick();
  assert(ntick <= tick());  // detect underflow
  return create(ntick << EPSILON_BITS);
}

Time Time::operator-(Tick _tick) const {
  Tick ntick = tick() - _tick;
  assert(ntick <= tick());  // detect underflow
  return create(ntick << EPSILON_BITS);
}

Time& Time::operator+=(const Time& _o) {
  Tick ntick = tick() + _o.tick();
  assert(ntick >= tick());  // detect overflow
  timeStep_ = ntick << EPSILON_BITS;
  return *this;
}

Time& Time::operator+=(Tick _tick) {
  Tick ntick = tick() + _tick;
  assert(ntick >= tick());  // detect overflow
  timeStep_ = ntick << EPSILON_BITS;
  return *this;
}

Time& Time::operator-=(const Time& _o) {
  Tick ntick = tick() - _o.tick();
  assert(ntick <= tick());  // detect underflow
  timeStep_ = ntick << EPSILON_BITS;
  return *this;
}

Time& Time::operator-=(Tick _tick) {
  Tick ntick = tick() - _tick;
  assert(ntick <= tick());  // detect underflow
  timeStep_ = ntick << EPSILON_BITS;
  return *this;
}

Time Time::nextEpsilon() const {
  assert(epsilon() < EPSILON_INV);
  return create(timeStep_ + 1);
}

void Time::incrEpsilon() {
  assert(epsilon() < EPSILON_INV);
  timeStep_ += 1;
}

std::string Time::toString() const {
  std::stringstream ss;
  ss << static_cast<u64>(tick()) << ':' << static_cast<u64>(epsilon());
  return ss.str();
}

bool Time::valid() const {
  return tick() != TICK_INV && epsilon() != EPSILON_INV;
}

TimeStep Time::raw() const {
  return timeStep_;
}

}  // namespace des
