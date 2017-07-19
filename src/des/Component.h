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
#ifndef DES_COMPONENT_H_
#define DES_COMPONENT_H_

#include <prim/prim.h>

#include <string>

#include "des/Time.h"

namespace des {

class Logger;
class Simulator;

class Component {
 public:
  // this constructor is for a top level component
  Component(Simulator* _simulator, const std::string& _name);

  // this constructor is for a child component
  Component(const std::string& _name, const Component* _parent);

  virtual ~Component();
  const std::string& basename() const;
  std::string fullname() const;
  const Component* parent() const;
  u32 executer() const;
  Component* self() const;

  mutable Simulator* simulator;  // mutable to call addEvent
  bool debug;

 private:
  // this the full constructor
  Component(Simulator* _simulator, const std::string& _name,
            const Component* _parent);

  std::string basename_;
  const Component* parent_;
  u32 executer_;

  friend class Simulator;
};

#ifndef NDEBUGLOG
s32 debuglogf(Logger* _logger, const char* _func, s32 _line, const char* _name,
              const Time& _time, const char* _format, ...);

#define dlogf(...) (                                                    \
    ((debug) && (simulator->getLogger())) ?                             \
    (des::debuglogf(simulator->getLogger(),                             \
                    __func__,                                           \
                    __LINE__,                                           \
                    fullname().c_str(),                                 \
                    simulator->time(),                                  \
                    __VA_ARGS__)) : (0))
#else  // NDEBUGLOG

#define dlogf(...)

#endif  // NDEBUGLOG

}  // namespace des

#endif  // DES_COMPONENT_H_
