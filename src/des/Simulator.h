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
#ifndef DES_SIMULATOR_H_
#define DES_SIMULATOR_H_

#include <prim/prim.h>

#include <atomic>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "des/SpinLock.h"

namespace des {

class Event;
class Executer;
class Model;
class Logger;

class Simulator {
 public:
  explicit Simulator(u32 _numThreads);
  ~Simulator();

  // events and event handling
  u64 time() const;
  u8 epsilon() const;
  void addEvent(Event* _event);
  u64 numEvents() const;
  void simulate(bool _printStatsSummary);
  void showStats();

  // logging
  Logger* getLogger() const;
  void setLogger(Logger* _logger);

  // models and debugging
  void addModel(Model* _model);
  Model* getModel(const std::string& _fullName) const;
  void removeModel(const std::string& _fullName);
  u64 numModels() const;
  void addDebugName(const std::string& _fullName);
  void debugCheck();

 private:
  std::vector<Executer*> executers_;
  volatile std::atomic<u64> working_;  // DO I NEED VOLATILE HERE?

  // following variables handle time
  u64 time_;
  u8 epsilon_;

  // this is used to tell the simulation thread to print current stats
  volatile bool showStats_;

  // this is a logger for the entire simulation framework
  Logger* logger_;

  // models and debugging
  std::unordered_map<std::string, Model*> models_;
  std::unordered_set<std::string> toBeDebugged_;
};

}  // namespace des

#endif  // DES_SIMULATOR_H_
