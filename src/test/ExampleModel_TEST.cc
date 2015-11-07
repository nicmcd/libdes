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
#include "test/ExampleModel_TEST.h"

#include <cassert>
#include <cstdio>
#include <cstring>

ExampleModel::ExampleModel(des::Simulator* _simulator, const std::string& _name,
                           const Model* _parent, u64 _count, u64 _id,
                           bool _verbose)
    : des::Model(_simulator, _name, _parent), count_(_count),
      id_(_id), verbose_(_verbose), evt_(this, static_cast<des::EventHandler>(
          &ExampleModel::exampleFunctionHandler)) {}

ExampleModel::~ExampleModel() {}

ExampleModel::ExampleEvent::ExampleEvent(des::Model* _model,
                                         des::EventHandler _handler)
    : des::Event(_model, _handler), a(0), b(0), c(0) {}

void ExampleModel::exampleFunction(s32 _a, s32 _b, s32 _c) {
  evt_.time = simulator->time();
  evt_.time.tick++;
  evt_.time.epsilon = count_ % 254;
  evt_.a = _a;
  evt_.b = _b;
  evt_.c = _c;
  simulator->addEvent(&evt_);
}

void ExampleModel::exampleFunctionHandler(des::Event* _event) {
  ExampleEvent* me = dynamic_cast<ExampleEvent*>(_event);

  count_--;
  if (verbose_ || count_ < 5) {
    // logf("hello world, from model #%lu, count %lu", id_, count_);
  }

  if (count_ > 0) {
    s32 na = me->a + 1;
    s32 nb = me->b + 1;
    s32 nc = me->c + 1;
    exampleFunction(na, nb, nc);  // queue another
  }
}
