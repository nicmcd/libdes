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
#ifndef DES_MODEL_H_
#define DES_MODEL_H_

#include <prim/prim.h>

#include <string>

namespace des {

class Simulator;

class Model {
 public:
  // this constructor inherits the Simulator from the parent
  Model(const std::string& _name, const Model* _parent);
  // this is the full constructor
  Model(Simulator* _simulator, const std::string& _name,
        const Model* _parent);
  virtual ~Model();
  const std::string& baseName() const;
  std::string fullName() const;
  void setDebug(bool _debug);

 protected:
  Simulator* simulator() const;
  bool debug_;

 private:
  Simulator* simulator_;
  std::string name_;
  const Model* parent_;
};

#ifndef NDEBUGLOG
s32 debuglogf(const char* _func, s32 _line, const char* _name,
              u64 _time, u8 _epsilon, const char* _format, ...);

#define logf(...) (                                                     \
    (this->debug_) ?                                                    \
    (des::debuglogf(__func__,                                           \
                    __LINE__,                                           \
                    this->fullName().c_str(),                           \
                    simulator()->time(),                                \
                    simulator()->epsilon(),                             \
                    __VA_ARGS__)) : (0))
#else  // NDEBUGLOG

#define logf(...)

#endif  // NDEBUGLOG

}  // namespace des

#endif  // DES_MODEL_H_
