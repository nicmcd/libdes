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
#include "des/Clocked.h"

#include <gtest/gtest.h>
#include <prim/prim.h>

#include <chrono>
#include <thread>

#include "des/Event.h"
#include "des/Component.h"
#include "des/Simulator.h"
#include "des/Time.h"

class SetTimeComponent : public des::Component {
 public:
  explicit SetTimeComponent(des::Simulator* _simulator)
      : des::Component(_simulator, "settime", nullptr),
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

u64 slowFutureCycle(des::Tick _now, des::Tick _period, des::Tick _phase,
                    u64 _cycles) {
  des::Tick tick = _now + 1;
  while (tick % _period != _phase) {
    tick++;
  }
  tick += (_period * (_cycles - 1));
  return tick;
}

TEST(Clocked, futureCycle) {
  const u64 SIMS = 100;

  des::Simulator sim;
  des::Clocked a(&sim, "a", nullptr, 1000, 0);
  des::Clocked b("b", &a);
  des::Clocked c("c", &a, 1500, 500);
  std::vector<des::Clocked*> m({&a, &b, &c});
  SetTimeComponent t(&sim);

  ASSERT_EQ(a.baseName(), "a");
  ASSERT_EQ(a.fullName(), "a");
  ASSERT_EQ(a.cyclePeriod(), 1000u);
  ASSERT_EQ(a.cyclePhase(), 0u);

  ASSERT_EQ(b.baseName(), "b");
  ASSERT_EQ(b.fullName(), "a.b");
  ASSERT_EQ(b.cyclePeriod(), 1000u);
  ASSERT_EQ(b.cyclePhase(), 0u);

  ASSERT_EQ(c.baseName(), "c");
  ASSERT_EQ(c.fullName(), "a.c");
  ASSERT_EQ(c.cyclePeriod(), 1500u);
  ASSERT_EQ(c.cyclePhase(), 500u);

  for (u64 cnt = 0; cnt < SIMS; cnt++) {
    des::Tick now = sim.time().tick();
    for (u64 cyc = 1; cyc < 5; cyc++) {
      for (u64 idx = 0; idx < m.size(); idx++) {
        des::Clocked* cm = m.at(idx);
        des::Tick period = cm->cyclePeriod();
        des::Tick phase = cm->cyclePhase();
        des::Tick cmfc = cm->futureCycle(cyc);
        des::Tick sfc = slowFutureCycle(now, period, phase, cyc);
        ASSERT_EQ(cmfc, sfc);
      }
    }
    t.setEvent(des::Time(now + 1, 12));
    sim.simulate();
  }
}
