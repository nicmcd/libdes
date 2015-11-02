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
#include "des/Model.h"

#include <cstdarg>

#include "des/Simulator.h"

namespace des {

Model::Model(const std::string& _name, const Model* _parent)
    : Model(_parent->simulator_, _name, _parent) {}

Model::Model(Simulator* _simulator, const std::string& _name,
             const Model* _parent)
    : debug_(false), simulator_(_simulator), name_(_name),
      parent_(_parent) {
  simulator_->addModel(this);
}

Model::~Model() {
  simulator_->removeModel(fullName());
}

const std::string& Model::baseName() const {
  return name_;
}

std::string Model::fullName() const {
  if (parent_) {
    std::string name(parent_->fullName());
    name += '.';
    name += name_;
    return name;
  } else {
    return name_;
  }
}

void Model::setDebug(bool _debug) {
  debug_ = _debug;
}

Simulator* Model::simulator() const {
  return simulator_;
}

#ifndef NDEBUGLOG
s32 debuglogf(const char* _func, s32 _line, const char* _name,
              u64 _time, u8 _epsilon, const char* _format, ...) {
  // this will cause errors with multiple threads!!!
  printf("[%lu:%u] %s:%s:%i | ", _time, _epsilon, _name, _func, _line);
  va_list args;
  va_start(args, _format);
  vprintf(_format, args);
  printf("\n");
  va_end(args);
  return 0;
}
#endif  // NDEBUGLOG

}  // namespace des
