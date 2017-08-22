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
#include "des/util/Scheduler.h"

#include <gtest/gtest.h>

#include <random>
#include <string>
#include <unordered_set>

#include "des/ActiveComponent.h"
#include "des/Component.h"
#include "des/Logger.h"
#include "des/Mapper.h"
#include "des/Simulator.h"
#include "des/Time.h"
#include "des/util/RoundRobinMapper.h"

namespace {
class Scheduled : public des::ActiveComponent {
 public:
  Scheduled(des::Simulator* _sim, const std::string& _name,
            std::unordered_set<des::Time>* _exeTimes)
      : des::ActiveComponent(_sim, _name),
        scheduler_(this, makeHandler(Scheduled, processRequests)),
        exeTimes_(_exeTimes) {
    debug = false;
  }

  void request() const {
    dlogf("request");
    scheduler_.schedule(simulator->time().nextEpsilon());
  }

  void processRequests(des::Event* _event) {
    dlogf("processing");
    scheduler_.executed();

    (void)_event;  // unused

    // make sure future only
    if (lastTime_.valid()) {
      assert(simulator->time() > lastTime_);
    }

    // unmark this time
    assert(exeTimes_->erase(simulator->time()) == 1);

    // save for next time
    lastTime_ = simulator->time();
  }

 private:
  des::Scheduler<des::Event> scheduler_;
  des::Time lastTime_;
  std::unordered_set<des::Time>* exeTimes_;
};

class Driver : public des::ActiveComponent {
 public:
  Driver(des::Simulator* _sim, const std::string& _name,
         Scheduled* _scheduled)
      : des::ActiveComponent(_sim, _name), scheduled_(_scheduled) {}

  void createRequest(des::Time _time) const {
    simulator->addEvent(new des::Event(
        self(), makeHandler(Driver, performRequest), _time));
  }

 private:
  void performRequest(des::Event* _event) {
    scheduled_->request();
    delete _event;
  }

  Scheduled* scheduled_;
};

TEST(Scheduler, basic) {
  std::mt19937 rnd;


  des::Simulator sim(10);
  des::RoundRobinMapper mapper;
  sim.setMapper(&mapper);
  des::Logger logger;
  sim.setLogger(&logger);

  std::unordered_set<des::Time> exeTimes;
  Scheduled sch(&sim, "Sch", &exeTimes);

  std::vector<Driver*> drivers;
  for (u32 d = 0; d < 1000; d++) {
    // create
    drivers.push_back(new Driver(&sim, "Driver_" + std::to_string(d), &sch));

    // make requests
    u32 num = rnd() % 1000;
    for (u32 r = 0; r < num; r++) {
      des::Time time = des::Time(rnd() % 10000, 0);
      drivers.at(d)->createRequest(time);
      exeTimes.insert(time.nextEpsilon());
    }
  }

  sim.initialize();
  sim.simulate();

  ASSERT_EQ(exeTimes.size(), 0u);

  for (u32 d = 0; d < 1000; d++) {
    delete drivers.at(d);
  }
}

}  // namespace
