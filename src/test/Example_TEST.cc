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
#include <gtest/gtest.h>
#include <prim/prim.h>

#include <cmath>

#include <tuple>
#include <vector>

#include "des/des.h"
#include "test/ExampleModel_TEST.h"

TEST(ExampleModel, simple) {
  u32 numThreads = 1;
  u64 numModels = 2000;
  printf("number of models: %lu\n", numModels);
  u64 eventsPerModel = 10000;

  des::Simulator* sim = new des::Simulator(numThreads);
  for (u32 id = 0; id < 1/*numModels*/; id++) {
    sim->addDebugName("Model_" + std::to_string(id));
  }

  std::vector<ExampleModel*> models(numModels);
  for (u32 id = 0; id < numModels; id++) {
    models.at(id) = new ExampleModel(sim, "Model_" + std::to_string(id),
                                     nullptr, eventsPerModel, id, false);
  }
  for (u32 id = 0; id < numModels; id++) {
    models.at(id)->exampleFunction(1000000, 2000000, 3000000);
  }

  sim->debugCheck();
  sim->simulate();

  for (u32 id = 0; id < numModels; id++) {
    delete models.at(id);
  }
  delete sim;
}

TEST(ExampleModel, benchmark) {
  const u64 configs[5][2] = {
    {1,     250000000},
    {10,    12000000},
    {100,   800000},
    {1000,  50000},
    {10000, 3200}};

  for (auto&& c : configs) {  // NOLINT
    u64 numModels = c[0];
    printf("number of models: %lu\n", numModels);
    u64 eventsPerModel = c[1];

    des::Simulator* sim = new des::Simulator();
    sim->addDebugName("Model_" + std::to_string(numModels - 1));

    std::vector<ExampleModel*> models(numModels);
    for (u32 id = 0; id < numModels; id++) {
      models.at(id) = new ExampleModel(sim, "Model_" + std::to_string(id),
                                       nullptr, eventsPerModel, id, false);
    }
    for (u32 id = 0; id < numModels; id++) {
      models.at(id)->exampleFunction(1000000, 2000000, 3000000);
    }

    sim->debugCheck();
    sim->simulate();

    for (u32 id = 0; id < numModels; id++) {
      delete models.at(id);
    }
    delete sim;
  }
}
