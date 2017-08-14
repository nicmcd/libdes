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
#include "des/Mapper.h"

#include <gtest/gtest.h>
#include <prim/prim.h>

#include "des/ActiveComponent.h"
#include "des/Simulator.h"

namespace {
class TestMapper : public des::Mapper {
 public:
  TestMapper() : count(0) {}
  ~TestMapper() {}
  u32 map(u32 _numExecuters, const des::ActiveComponent* _component) override {
    count++;
    return std::stoul(_component->basename()) % _numExecuters;
  }
  u32 count;
};

class TestComponent : public des::ActiveComponent {
 public:
  TestComponent(des::Simulator* _simulator, const std::string& _name)
      : des::ActiveComponent(_simulator, _name) {}
  ~TestComponent() {}
};
}  // namespace

TEST(Mapper, simulatorAssignment) {
  for (u32 numExecuters : std::vector<u32>({1, 2, 10, 100})) {
    // create simulator
    TestMapper* mapper = new TestMapper();
    des::Simulator* sim = new des::Simulator(numExecuters);
    sim->setMapper(mapper);

    // create components
    std::vector<des::ActiveComponent*> comps;
    const u32 kNumComps = 10000;
    for (u32 c = 0; c < kNumComps; c++) {
      comps.push_back(new TestComponent(sim, std::to_string(c)));
    }

    // verify component executer assignment
    for (u32 c = 0; c < kNumComps; c++) {
      u32 exp = std::stoull(comps.at(c)->basename()) % numExecuters;
      if (exp == 1) {
        ASSERT_EQ(comps.at(c)->executer(), exp);
      }
    }

    // verify mapper count
    if (numExecuters > 1) {
      ASSERT_EQ(mapper->count, kNumComps);
    } else {
      ASSERT_EQ(mapper->count, 0u);
    }

    // cleanup
    for (u32 c = 0; c < kNumComps; c++) {
      delete comps.at(c);
    }
    delete sim;
    delete mapper;
  }
}
