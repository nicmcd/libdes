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
#include "des/ActiveComponent.h"

#include <gtest/gtest.h>

#include "des/Mapper.h"
#include "des/Simulator.h"

class TestMapper : public des::Mapper {
 public:
  TestMapper() : last_(0) {}
  ~TestMapper() {}

  u32 map(u32 _numExecuters, const des::ActiveComponent* _component) override {
    (void)_component;  // unused
    return last_ = (last_ == 0) ? _numExecuters - 1 : last_ - 1;
  }

 private:
  u32 last_;
};

class TestComponent : public des::ActiveComponent {
 public:
  TestComponent(des::Simulator* _simulator, const std::string& _name)
      : des::ActiveComponent(_simulator, _name) {}
  ~TestComponent() {}

  bool checkSameExecutor(bool _expected) const {
    return _expected == sameExecuter();
  }

  void sameCheck(des::Time _time) const {
    simulator->addEvent(new des::Event(
        self(), makeHandler(TestComponent, processSameCheck),
        _time));
  }

 private:
  void processSameCheck(des::Event* _event) {
    delete _event;
    ASSERT_TRUE(sameExecuter());
  }
};

TEST(ActiveComponent, executer) {
  const u32 EXES = 4;
  const u32 COMPS = 50;

  des::Mapper* mapper = new TestMapper();
  des::Simulator* sim = new des::Simulator(EXES);
  sim->setMapper(mapper);

  // create components
  std::vector<TestComponent*> comps;
  for (u32 c = 0; c < COMPS; c++) {
    comps.push_back(new TestComponent(sim, std::to_string(c)));
  }

  // verify executer
  ASSERT_EQ(sim->executerId(), U32_MAX);
  u32 exp = EXES - 1;
  for (u32 c = 0; c < COMPS; c++) {
    TestComponent* comp = comps.at(c);
    ASSERT_EQ(comp->executer(), exp);
    exp = exp == 0 ? EXES - 1 : exp - 1;
  }

  // set some checks
  for (u32 c = 0; c < COMPS; c++) {
    comps.at(c)->sameCheck(des::Time(100, 0));
    comps.at(c)->sameCheck(des::Time(123, 2));
  }

  sim->initialize();
  sim->simulate();

  // delete components
  for (u32 c = 0; c < COMPS; c++) {
    delete comps.at(c);
  }

  delete sim;
  delete mapper;
}
