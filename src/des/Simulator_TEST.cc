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
#include "des/Simulator.h"

#include <gtest/gtest.h>
#include <prim/prim.h>

#include <random>
#include <vector>

#include "des/Event.h"
#include "des/Model.h"
#include "des/Time.h"

class NullModel : public des::Model {
 public:
  NullModel(des::Simulator* _simulator, u64 _id)
      : des::Model(_simulator, std::to_string(_id), nullptr), id_(_id),
        event_(this, static_cast<des::EventHandler>(
            &NullModel::ignoreEvent), des::Time()) {
    debug = true;
  }

  void ignoreEvent(des::Event* _event) {
    (void)_event;
  }

  void nextEvent() {
    event_.time.tick += 1 + (prng_() % 100);
    simulator->addEvent(&event_);
  }

 private:
  u64 id_;
  des::Event event_;
  std::mt19937_64 prng_;
};

TEST(Simulator, multisim) {
  const u64 MODELS = 3;
  const u64 SIMS = 100;

  des::Simulator* sim = new des::Simulator();
  std::vector<NullModel*> models;
  for (u64 id = 0; id < MODELS; id++) {
    models.push_back(new NullModel(sim, id));
  }

  for (u64 s = 0; s < SIMS; s++) {
    for (u64 id = 0; id < MODELS; id++) {
      models.at(id)->nextEvent();
    }
    sim->simulate(false);
  }

  for (u64 id = 0; id < MODELS; id++) {
    delete models.at(id);
  }
  delete sim;
}
