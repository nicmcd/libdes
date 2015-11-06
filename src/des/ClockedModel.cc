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
#include "des/ClockedModel.h"

#include <cassert>

#include "des/Simulator.h"

namespace des {

ClockedModel::ClockedModel(const std::string& _name,
                           const ClockedModel* _parent)
    : ClockedModel(_parent->simulator, _name, _parent, _parent->cyclePeriod_,
                   _parent->cyclePhase_) {}

ClockedModel::ClockedModel(const std::string& _name,
                           const ClockedModel* _parent, Tick _cyclePeriod,
                           Tick _cyclePhase)
    : ClockedModel(_parent->simulator, _name, _parent, _cyclePeriod,
                   _cyclePhase) {}

ClockedModel::ClockedModel(Simulator* _simulator, const std::string& _name,
                           const Model* _parent, Tick _cyclePeriod,
                           Tick _cyclePhase)
    : Model(_simulator, _name, _parent), cyclePeriod_(_cyclePeriod),
      cyclePhase_(_cyclePhase) {
  assert(cyclePhase_ < cyclePeriod_);
}

ClockedModel::~ClockedModel() {}

Tick ClockedModel::cyclePeriod() const {
  return cyclePeriod_;
}

Tick ClockedModel::cyclePhase() const {
  return cyclePhase_;
}

Tick ClockedModel::futureCycle(u32 _cycles) const {
  assert(_cycles > 0);
  Tick tick = simulator->time().tick;
  Tick rem = tick % cyclePeriod_;
  if (rem != cyclePhase_) {
    tick += (cyclePhase_ - rem);
    if (rem > cyclePhase_) {
      tick += cyclePeriod_;
    }
    _cycles--;
  }
  return tick + (cyclePeriod_ * _cycles);
}

}  // namespace des
