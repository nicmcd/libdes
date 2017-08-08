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

#include "des/Event.h"
#include "des/Component.h"
#include "des/Logger.h"
#include "des/Time.h"
#include "des/util/RoundRobinMapper.h"

class TestComponent : public des::Component {
 public:
  TestComponent(const std::string& _name, const Component* _parent)
      : des::Component(_name, _parent) {}
  TestComponent(des::Simulator* _simulator, const std::string& _name)
      : des::Component(_simulator, _name) {}
  ~TestComponent() {}
};

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

  sim.debugCheck();
}

TEST(Simulator, numComponents) {
  des::Simulator sim;

  TestComponent tc1(&sim, "top");
  TestComponent tc2("left", &tc1);
  TestComponent tc3("right", &tc1);
  ASSERT_EQ(sim.numComponents(), 3u);
}

class NullComponent : public des::Component {
 public:
  NullComponent(des::Simulator* _simulator, u64 _id)
      : des::Component(_simulator, std::to_string(_id)), id_(_id),
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

TEST(Simulator, emptysim) {
  const u64 COMPONENTS = 3;
  const u64 SIMS = 100;

  des::Simulator* sim = new des::Simulator();
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

  des::Simulator* sim = new des::Simulator();
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

class OneAtZero : public des::Component {
 public:
  OneAtZero(des::Simulator* _simulator, u64 _id)
      : des::Component(_simulator, std::to_string(_id)), id_(_id),
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

TEST(Simulator, execTime0) {
  const u64 COMPONENTS = 3;
  const u64 SIMS = 100;

  des::Simulator* sim = new des::Simulator();
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

class SubSimulator : public des::Simulator {
 public:
  explicit SubSimulator(u32 _numExecuters, des::Mapper* _mapper)
      : des::Simulator(_numExecuters, _mapper) {
    balances.resize(_numExecuters, 0);
  }
  ~SubSimulator() {}
  s64& balance() {
    // return balances.at(0);  // THIS IS AN EXAMPLE OF BREAKING IT!
    return balances.at(executerId());
  }
  std::vector<s64> balances;
};

class SubComponent : public des::Component {
 public:
  SubComponent(des::Simulator* _simulator, const std::string& _name)
      : des::Component(_simulator, _name), event_(this), count_(0) {
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
    explicit Event(des::Component* _component)
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

    sim->debugCheck();
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
