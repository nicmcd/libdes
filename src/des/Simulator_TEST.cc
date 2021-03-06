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
#include "des/Simulator.h"

#include <vector>

#include "des/ActiveComponent.h"
#include "des/Component.h"
#include "des/Event.h"
#include "des/Logger.h"
#include "des/Time.h"
#include "des/util/RoundRobinMapper.h"
#include "gtest/gtest.h"
#include "prim/prim.h"
#include "rnd/Random.h"

namespace {
class TestComponent : public des::Component {
 public:
  TestComponent(const std::string& _name, const Component* _parent)
      : des::Component(_name, _parent) {}
  TestComponent(des::Simulator* _simulator, const std::string& _name)
      : des::Component(_simulator, _name) {}
  ~TestComponent() {}
};
}  // namespace

TEST(Simulator, debug) {
  des::Simulator sim;
  sim.addDebugName("top");
  sim.addDebugName("top.right");

  TestComponent tc1(&sim, "top");
  ASSERT_TRUE(tc1.debug);
  TestComponent tc2("left", &tc1);
  ASSERT_FALSE(tc2.debug);
  TestComponent tc3("right", &tc1);
  ASSERT_TRUE(tc3.debug);

  sim.debugNameCheck();
}

TEST(Simulator, numComponents) {
  des::Simulator sim;

  TestComponent tc1(&sim, "top");
  TestComponent tc2("left", &tc1);
  TestComponent tc3("right", &tc1);
  ASSERT_EQ(sim.numComponents(), 3u);
}

namespace {
class NullComponent : public des::ActiveComponent {
 public:
  NullComponent(des::Simulator* _simulator, u64 _id)
      : des::ActiveComponent(_simulator, std::to_string(_id)),
        id_(_id),
        event_(this, std::bind(&NullComponent::ignoreEvent, this), des::Time()),
        prng_(123) {
    debug = true;
  }

  void ignoreEvent() {}

  void nextEvent() {
    event_.time += 1 + prng_.nextU64(0, 101);
    simulator->addEvent(&event_);
  }

 private:
  u64 id_;
  des::Event event_;
  rnd::Random prng_;
};
}  // namespace

TEST(Simulator, emptysim) {
  const u64 COMPONENTS = 3;
  const u64 SIMS = 100;

  des::Simulator* sim = new des::Simulator(1);
  std::vector<NullComponent*> components;
  for (u64 id = 0; id < COMPONENTS; id++) {
    components.push_back(new NullComponent(sim, id));
  }

  for (u64 s = 0; s < SIMS; s++) {
    sim->simulate();
  }

  for (u64 id = 0; id < COMPONENTS; id++) {
    delete components.at(id);
  }
  delete sim;
}

TEST(Simulator, multisim) {
  const u64 COMPONENTS = 3;
  const u64 SIMS = 100;

  des::Simulator* sim = new des::Simulator(1);
  std::vector<NullComponent*> components;
  for (u64 id = 0; id < COMPONENTS; id++) {
    components.push_back(new NullComponent(sim, id));
  }

  for (u64 s = 0; s < SIMS; s++) {
    for (u64 id = 0; id < COMPONENTS; id++) {
      components.at(id)->nextEvent();
    }
    sim->simulate();
  }

  for (u64 id = 0; id < COMPONENTS; id++) {
    delete components.at(id);
  }
  delete sim;
}

namespace {
class OneAtZero : public des::ActiveComponent {
 public:
  OneAtZero(des::Simulator* _simulator, u64 _id)
      : des::ActiveComponent(_simulator, std::to_string(_id)),
        id_(_id),
        event_(this, std::bind(&OneAtZero::ignoreEvent, this), des::Time()) {
    debug = true;
    event_.time = 0;
    simulator->addEvent(&event_);
  }

  void ignoreEvent() {}

 private:
  u64 id_;
  des::Event event_;
};
}  // namespace

TEST(Simulator, execTime0) {
  const u64 COMPONENTS = 3;
  const u64 SIMS = 100;

  des::Simulator* sim = new des::Simulator(1);
  std::vector<OneAtZero*> components;
  for (u64 id = 0; id < COMPONENTS; id++) {
    components.push_back(new OneAtZero(sim, id));
  }

  for (u64 s = 0; s < SIMS; s++) {
    sim->simulate();
  }

  for (u64 id = 0; id < COMPONENTS; id++) {
    delete components.at(id);
  }
  delete sim;
}

namespace {
class SubSimulator : public des::Simulator {
 public:
  explicit SubSimulator(u32 _numExecuters, des::Mapper* _mapper)
      : des::Simulator(_numExecuters) {
    setMapper(_mapper);
    balances.resize(_numExecuters, 0);
  }
  ~SubSimulator() {}
  s64& balance() {
    // return balances.at(0);  // THIS IS AN EXAMPLE OF BREAKING IT!
    return balances.at(executerId());
  }
  std::vector<s64> balances;
};

class SubComponent : public des::ActiveComponent {
 public:
  SubComponent(des::Simulator* _simulator, const std::string& _name)
      : des::ActiveComponent(_simulator, _name),
        event_(this, std::bind(&SubComponent::handleEvent, this)),
        count_(0) {
    nextEvent();
  }
  ~SubComponent() {}

  void nextEvent() const {
    event_.time = simulator->time() + 1;
    simulator->addEvent(&event_);
  }

 private:
  void handleEvent() {
    count_++;
    // dlogf("count is %u", count_);
    if (count_ < 1000) {
      nextEvent();
    }

    s64& balance = reinterpret_cast<SubSimulator*>(simulator)->balance();
    balance += count_ % 2 == 0 ? 1 : -1;
  }

  mutable des::Event event_;
  u32 count_;
};
}  // namespace

TEST(Simulator, inheritRaceFreeThreadLocal) {
  for (u32 executers = 1; executers < 8; executers++) {
    des::Logger logger("-");
    des::Mapper* mapper = new des::RoundRobinMapper();
    des::Simulator* sim = new SubSimulator(executers, mapper);
    sim->setLogger(&logger);

    std::vector<des::Component*> comps;
    for (u32 cnt = 0; cnt < 10; cnt++) {
      std::string name = "Comp_" + std::to_string(cnt);
      sim->addDebugName(name);
      comps.push_back(new SubComponent(sim, name));
      ASSERT_TRUE(comps.at(cnt)->debug);
    }

    sim->debugNameCheck();
    sim->simulate();

    for (u32 t = 0; t < executers; t++) {
      s64 bal = reinterpret_cast<SubSimulator*>(sim)->balances.at(t);
      ASSERT_EQ(bal, 0);
    }

    for (u32 cnt = 0; cnt < 10; cnt++) {
      delete comps.at(cnt);
    }

    delete sim;
    delete mapper;
  }
}

namespace {
class SetTimeComponent : public des::ActiveComponent {
 public:
  explicit SetTimeComponent(des::Simulator* _simulator)
      : des::ActiveComponent(_simulator, "settime"),
        event_(this, std::bind(&SetTimeComponent::ignoreEvent, this)) {
    debug = true;
  }

  void setEvent(des::Time _time) {
    event_.time = _time;
    simulator->addEvent(&event_);
  }

  void ignoreEvent() {}

 private:
  des::Event event_;
};
}  // namespace

u64 slowFutureCycle(des::Tick _now, des::Tick _period, des::Tick _phase,
                    u64 _cycles) {
  des::Tick tick = _now + 1;
  while (tick % _period != _phase) {
    tick++;
  }
  tick += (_period * (_cycles - 1));
  return tick;
}

TEST(Simulator, futureCycle) {
  const u64 SIMS = 100;

  des::Simulator sim(1);
  u32 clock1 = sim.addClock(1000, 0);
  u32 clock2 = sim.addClock(1500, 500);
  std::vector<u32> clocks({clock1, clock2});

  SetTimeComponent t(&sim);

  ASSERT_EQ(sim.clockPeriod(clock1), 1000u);
  ASSERT_EQ(sim.clockPhase(clock1), 0u);

  ASSERT_EQ(sim.clockPeriod(clock2), 1500u);
  ASSERT_EQ(sim.clockPhase(clock2), 500u);

  for (u64 cnt = 0; cnt < SIMS; cnt++) {
    des::Tick now = sim.time().tick();
    for (u64 cyc = 1; cyc < 5; cyc++) {
      for (u32 clock : clocks) {
        for (des::Epsilon e = 0; e < 3; e++) {
          des::Tick period = sim.clockPeriod(clock);
          des::Tick phase = sim.clockPhase(clock);
          u64 cycle = sim.cycle(clock);
          ASSERT_EQ(cycle, (now + phase) / period);
          des::Time time;
          if (e == 0) {
            time = sim.futureCycle(clock, cyc);
          } else {
            time = sim.futureCycle(clock, cyc, e);
          }
          ASSERT_EQ(time.epsilon(), e);
          des::Tick cmfc = time.tick();
          des::Tick sfc = slowFutureCycle(now, period, phase, cyc);
          // printf("period=%lu phase=%lu now=%lu cycles=%lu, cmfc=%lu\n",
          //        period, phase, now, cyc, cmfc);
          ASSERT_EQ(cmfc, sfc);
        }
      }
    }
    t.setEvent(des::Time(now + 1, 12));
    sim.simulate();
  }
}

namespace {
class CycleCheckComponent : public des::ActiveComponent {
 public:
  explicit CycleCheckComponent(des::Simulator* _simulator)
      : des::ActiveComponent(_simulator, "CycleCheck") {
    debug = true;
  }

  void setCheck(des::Time _time, u32 _clockId, bool _expected) const {
    simulator->addEvent(new des::Event(
        self(),
        std::bind(&CycleCheckComponent::check,
                  const_cast<CycleCheckComponent*>(this), _clockId, _expected),
        _time, true));
  }

  void check(u32 _clockId, bool _expected) {
    bool actual = simulator->isCycle(_clockId);
    if (actual != _expected) {
      printf("time=%s clock=%u actual=%u expected=%u\n",
             simulator->time().toString().c_str(), _clockId, actual, _expected);
    }
    ASSERT_EQ(actual, _expected);
  }
};
}  // namespace

TEST(Simulator, isCycle) {
  des::Simulator sim(1);
  u32 clock1 = sim.addClock(1000, 0);
  u32 clock2 = sim.addClock(1500, 500);

  CycleCheckComponent checker(&sim);

  for (des::Epsilon e = 0; e < 4; e++) {
    checker.setCheck(des::Time(0, e), clock1, true);
    checker.setCheck(des::Time(0, e), clock2, false);

    checker.setCheck(des::Time(1, e), clock1, false);
    checker.setCheck(des::Time(1, e), clock2, false);

    checker.setCheck(des::Time(500, e), clock1, false);
    checker.setCheck(des::Time(500, e), clock2, true);

    checker.setCheck(des::Time(1000, e), clock1, true);
    checker.setCheck(des::Time(1000, e), clock2, false);

    checker.setCheck(des::Time(1500, e), clock1, false);
    checker.setCheck(des::Time(1500, e), clock2, false);

    checker.setCheck(des::Time(2000, e), clock1, true);
    checker.setCheck(des::Time(2000, e), clock2, true);

    checker.setCheck(des::Time(2500, e), clock1, false);
    checker.setCheck(des::Time(2500, e), clock2, false);

    checker.setCheck(des::Time(3000, e), clock1, true);
    checker.setCheck(des::Time(3000, e), clock2, false);

    checker.setCheck(des::Time(3500, e), clock1, false);
    checker.setCheck(des::Time(3500, e), clock2, true);
  }

  sim.simulate();
}

namespace {
struct UseRandomComponent : public des::ActiveComponent {
  rnd::Random* sim_random;
  std::vector<u64> values;

  UseRandomComponent(des::Simulator* _simulator, const std::string& _name)
      : des::ActiveComponent(_simulator, _name) {}

  void generateEvents(u32 _num_events) {
    sim_random = simulator->random();
    // Each event is generated in sequential ticks with random epsilons.
    // This tests the executer-level reproducibility.
    for (u32 e = 0; e < _num_events; e++) {
      des::Time time(simulator->time().tick() + e,
                     sim_random->nextU64() % des::EPSILON_INV);
      simulator->addEvent(new des::Event(
          this, std::bind(&UseRandomComponent::getRandomValue, this), time,
          true));
    }
  }

  void getRandomValue() {
    ASSERT_NE(simulator->random(), sim_random);
    values.push_back(simulator->random()->nextU64());
  }
};
}  // namespace

TEST(Simulator, random) {
  const u32 executers = 4;
  const u32 randoms = 400;
  const u32 check_sims = 100;
  des::Simulator sim(executers);
  des::RoundRobinMapper mapper;
  sim.setMapper(&mapper);
  std::vector<UseRandomComponent*> components(executers, nullptr);
  for (u32 c = 0; c < components.size(); c++) {
    components.at(c) = new UseRandomComponent(&sim, std::to_string(c));
  }

  // Runs one simulation to set the baseline values.
  sim.seed(12345678);
  for (u32 c = 0; c < components.size(); c++) {
    components.at(c)->generateEvents(randoms);
  }
  sim.simulate();

  // Collects the values to compare with later values.
  std::vector<std::vector<u64>> values(executers);
  for (u32 c = 0; c < components.size(); c++) {
    values.at(c) = components.at(c)->values;
    components.at(c)->values.clear();
  }

  // Runs more simulations and compares the values with the previously generated
  // values.
  for (u32 s = 0; s < check_sims; s++) {
    // Re-seeds the simulator, generates more events, runs the simulator.
    sim.seed(12345678);
    for (u32 c = 0; c < components.size(); c++) {
      components.at(c)->generateEvents(randoms);
    }
    sim.simulate();

    // Ensures the values match.
    for (u32 c = 0; c < components.size(); c++) {
      ASSERT_EQ(components.at(c)->values, values.at(c));
      components.at(c)->values.clear();
    }
  }

  for (u32 c = 0; c < components.size(); c++) {
    delete components.at(c);
  }
}
