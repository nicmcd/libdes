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
#ifndef DES_TIMECONF_H_
#define DES_TIMECONF_H_

#include <climits>

#include "prim/prim.h"

namespace des {

// this is the only thing that should ever be changed in this file
static const u64 EPSILON_BITS = 4;

// DO NOT EDIT BELOW THIS
static_assert(EPSILON_BITS > 0 && EPSILON_BITS < 64, "Invalid EPSILON_BITS");
typedef u64 TimeStep;
static const u64 TIMESTEP_INV = ~0;
static const u64 TIMESTEP_BITS = sizeof(TimeStep) * CHAR_BIT;
static const u64 EPSILON_INV = (1lu << EPSILON_BITS) - 1;
static const u64 TICK_BITS = TIMESTEP_BITS - EPSILON_BITS;
static const u64 TICK_INV = (1lu << TICK_BITS) - 1;

}  // namespace des

#endif  // DES_TIMECONF_H_
