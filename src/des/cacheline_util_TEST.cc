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
#include "des/cacheline_util.h"

#include <gtest/gtest.h>
#include <prim/prim.h>

TEST(cacheline_util, CLPAD_Logic) {
  ASSERT_EQ(des::CLPAD(1), des::CACHELINE_SIZE - 1);
  ASSERT_EQ(des::CLPAD(2 * des::CACHELINE_SIZE), 0u);
  ASSERT_EQ(des::CLPAD(2 * des::CACHELINE_SIZE + 1), des::CACHELINE_SIZE - 1);
  ASSERT_EQ(des::CLPAD(3 * des::CACHELINE_SIZE - 1), 1u);
}

TEST(cacheline_util, CLPAD_Use) {
  struct alignas(64) AlignedAndPaddedU64 {
    u64 num;
    char pad[des::CLPAD(sizeof(num))];
  };
  AlignedAndPaddedU64 aapu64;
  u64 u;

  // write
  aapu64.num = 0xABCDABCDABCDABCDllu;
  for (char& c : aapu64.pad) {
    c = 0;
  }
  u = 0xCDABCDABCDABCDABllu;;

  // check
  ASSERT_EQ(aapu64.num, 0xABCDABCDABCDABCDllu);
  for (char& c : aapu64.pad) {
    ASSERT_EQ(c, 0);
  }
  ASSERT_EQ(u, 0xCDABCDABCDABCDABllu);

  // size and address
  ASSERT_EQ(sizeof(aapu64), des::CACHELINE_SIZE);
  auto* ptr = &aapu64;
  u64 mod = reinterpret_cast<u64>(ptr) % des::CACHELINE_SIZE;
  ASSERT_EQ(mod, 0u);
}
