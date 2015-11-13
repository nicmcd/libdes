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
#include "des/SpinLock.h"

#include <gtest/gtest.h>
#include <prim/prim.h>

#include <chrono>
#include <ratio>


TEST(SpinLock, speedTest) {
  des::SpinLock lock;
  u64 randnum = 0x1;

  const u64 SEQUENCES = 20000000;
  std::chrono::steady_clock::time_point t1 =
      std::chrono::steady_clock::now();

  for (volatile u64 count = 0; count < SEQUENCES; count++) {
    lock.lock();
    randnum ^= (randnum << 23);
    lock.unlock();
  }

  std::chrono::steady_clock::time_point t2 =
      std::chrono::steady_clock::now();

  for (volatile u64 count = 0; count < SEQUENCES; count++) {
    randnum ^= (randnum << 23);
  }

  std::chrono::steady_clock::time_point t3 =
      std::chrono::steady_clock::now();

  f64 d1 = std::chrono::duration_cast<std::chrono::duration<f64> >(
      t2 - t1).count();
  f64 d2 = std::chrono::duration_cast<std::chrono::duration<f64> >(
      t3 - t2).count();

  f64 time = d1 - d2;

  printf("total time         : %f\n"
         "overhead time      : %f\n"
         "total sequences    : %lu\n"
         "spinlock time      : %f\n"
         "sequences/second   : %lu\n",
         d1, d2, SEQUENCES, time, (u64)(SEQUENCES / time));
}
