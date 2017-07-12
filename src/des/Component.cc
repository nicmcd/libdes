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
#include "des/Component.h"

#include <cassert>
#include <cstdarg>

#include "des/Logger.h"
#include "des/Simulator.h"

namespace des {

Component::Component(const std::string& _name, const Component* _parent)
    : Component(_parent->simulator, _name, _parent) {}

Component::Component(Simulator* _simulator, const std::string& _name,
                     const Component* _parent)
    : simulator(_simulator), debug(false), executer(U32_MAX),
      name_(_name), parent_(_parent) {
  simulator->addComponent(this);
}

Component::~Component() {
  simulator->removeComponent(fullName());
}

const std::string& Component::baseName() const {
  return name_;
}

std::string Component::fullName() const {
  if (parent_) {
    std::string name(parent_->fullName());
    name += '.';
    name += name_;
    return name;
  } else {
    return name_;
  }
}

#ifndef NDEBUGLOG
s32 debuglogf(Logger* _logger, const char* _func, s32 _line, const char* _name,
              const Time& _time, const char* _format, ...) {
  // create a buffer to create the string in
  s32 budget = 2000;
  char* buf = new char[budget];
  char* ptr = buf;
  s32 res;

  // format the line header
  res = snprintf(ptr, budget, "[%s] %s:%s:%i | ", _time.toString().c_str(),
                 _name, _func, _line);
  assert((res >= 0) && (res < budget));
  ptr += res;
  budget -= res;

  // add the rest of the contents
  va_list args;
  va_start(args, _format);
  res = vsnprintf(ptr, budget, _format, args);
  va_end(args);
  assert((res >= 0) && (res < budget));
  ptr += res;
  budget -= res;

  // add the line ending
  assert(budget >= 2);
  ptr[0] = '\n';
  ptr[1] = '\0';
  ptr += 2;

  // give string to logger to print
  _logger->log(buf, ptr - buf - 1);

  // delete the buffer and return succesfully
  delete[] buf;
  return 0;
}
#endif  // NDEBUGLOG

}  // namespace des
