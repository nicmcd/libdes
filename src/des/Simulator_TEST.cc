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

#include <gtest/gtest.h>
#include <prim/prim.h>

#include <random>
#include <vector>

#include "des/ActiveComponent.h"
#include "des/Component.h"
#include "des/Event.h"
#include "des/Logger.h"
#include "des/Time.h"
#include "des/util/RoundRobinMapper.h"

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
      : des::ActiveComponent(_simulator, std::to_string(_id)), id_(_id),
        event_(this, static_cast<des::EventHandler>(
            &NullComponent::ignoreEvent), des::Time()) {
    debug = true;
  }

  void ignoreEvent(des::Event* _event) {
    (void)_event;
  }

  void nextEvent() {
    event_.time += 1 + (prng_() % 100);
    simulator->addEvent(&event_);
  }

 private:
  u64 id_;
  des::Event event_;
  std::mt19937_64 prng_;
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

  sim->initialize();
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

  sim->initialize();
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
      : des::ActiveComponent(_simulator, std::to_string(_id)), id_(_id),
        event_(this, static_cast<des::EventHandler>(
            &OneAtZero::ignoreEvent), des::Time()) {
    debug = true;
    event_.time = 0;
    simulator->addEvent(&event_);
  }

  void ignoreEvent(des::Event* _event) {
    (void)_event;
  }

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

  sim->initialize();
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
      : des::ActiveComponent(_simulator, _name), event_(this), count_(0) {
    nextEvent();
  }
  ~SubComponent() {}

  void nextEvent() const {
    event_.time = simulator->time() + 1;
    simulator->addEvent(&event_);
  }

 private:
  class Event : public des::Event {
   public:
    explicit Event(des::ActiveComponent* _component)
        : des::Event(_component, static_cast<des::EventHandler>(
              &SubComponent::handleEvent)) {}
  };

  void handleEvent(des::Event* _event) {
    Event* event = reinterpret_cast<Event*>(_event);
    (void)event;

    count_++;
    // dlogf("count is %u", count_);
    if (count_ < 1000) {
      nextEvent();
    }

    s64& balance = reinterpret_cast<SubSimulator*>(simulator)->balance();
    balance += count_ % 2 == 0 ? 1 : -1;
  }

  mutable Event event_;
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
    sim->initialize();
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
        event_(this, static_cast<des::EventHandler>(
            &SetTimeComponent::ignoreEvent)) {
    debug = true;
  }

  void setEvent(des::Time _time) {
    event_.time = _time;
    simulator->addEvent(&event_);
  }

  void ignoreEvent(des::Event* _event) {
    (void)_event;
  }

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

  sim.initialize();
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
    simulator->addEvent(new Event(
        self(), makeHandler(CycleCheckComponent, check), _time,
        _clockId, _expected));
  }

  void check(des::Event* _event) {
    Event* event = reinterpret_cast<Event*>(_event);

    bool actual = simulator->isCycle(event->clockId);
    if (actual != event->expected) {
      printf("time=%s clock=%u actual=%u expected=%u\n",
             simulator->time().toString().c_str(), event->clockId,
             actual, event->expected);
    }
    ASSERT_EQ(actual, event->expected);

    delete event;
  }

 private:
  class Event : public des::Event {
   public:
    Event(des::ActiveComponent* _component, des::EventHandler _handler,
          des::Time _time, u32 _clockId, bool _expected)
        : des::Event(_component, _handler, _time),
          clockId(_clockId), expected(_expected) {}
    u32 clockId;
    bool expected;
  };
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

  sim.initialize();
  sim.simulate();
}
