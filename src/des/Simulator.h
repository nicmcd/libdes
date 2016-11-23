/*
 * Copyright (c) 2012-2016, Nic McDonald
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
 * - Neither the name of prim nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
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
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "des/Executer.h"
#include "des/Time.h"

namespace des {

class Event;
class Model;
class Logger;

class Simulator {
 public:
  Simulator();
  explicit Simulator(u32 _numThreads);
  ~Simulator();

  // events and event handling
  Time time() const;
  void addEvent(Event* _event);
  void simulate(bool _logSummary);

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

  // this triggers one statistics output in the logger
  void showStats();

 private:
  std::vector<std::tuple<Executer*, Executer::QueueStats> > executers_;

  // following variable holds the simulator's time
  Time time_;

  // this is a logger for the entire simulation framework
  Logger* logger_;

  // trigger to show stats
  volatile bool showStats_;

  // models and debugging
  std::unordered_map<std::string, Model*> models_;
  std::unordered_set<std::string> toBeDebugged_;
};

}  // namespace des

#endif  // DES_SIMULATOR_H_
