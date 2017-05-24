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
#include <chrono>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "des/cacheline_util.h"
#include "des/Event.h"
#include "des/MpScQueue.h"
#include "des/Time.h"

namespace des {

class Model;
class Logger;

// these variables are read/write shared variables for the executers
//  these are cache line aligned and padded individually
struct alignas(CACHELINE_SIZE) SimState {
  // this is an initial padding incase the object is dynamically allocated,
  //  which does not adhere to the alignas specifier
  char padding0[CACHELINE_SIZE];

  // this variable tells the executers to continue
  std::atomic<bool> running;
  char padding1[CLPAD(sizeof(running))];

  // this variable is the current time of execution
  TimeStep timeStep;
  char padding2[CLPAD(sizeof(timeStep))];

  // this variable is used by the executers to determine which epoch they are
  //  currently working in
  std::atomic<bool> epoch;
  char padding3[CLPAD(sizeof(epoch))];

  // this variable tracks how many executers have not yet hit the timestep
  //  barrier in the current epoch
  std::atomic<u32> yetToArrive;
  char padding4[CLPAD(sizeof(yetToArrive))];

  // this variable tracks the next time step to be used by epoch
  std::atomic<TimeStep> nextTimeStep[2];
  char padding5[CLPAD(sizeof(nextTimeStep))];

  // this variable is a flag to show statistics
  std::atomic<bool> showStats;
  char padding6[CLPAD(sizeof(showStats))];

  // this variable counts total number of executed events
  std::atomic<u64> eventCount;
  char padding7[CLPAD(sizeof(eventCount))];

  // this variable counts total number of executed time steps
  u64 timeStepCount;
  char padding8[CLPAD(sizeof(timeStepCount))];

  // this variable counts total number of executed ticks
  u64 tickCount;
  char padding9[CLPAD(sizeof(tickCount))];
};



// this structure contains variables that track performance statistics
/*
struct SimStats {
  u64 uniqueTimeSteps;
  u64 eventCount;
  u64 intervalEvents;
  Tick lastReportedTick;
  std::chrono::steady_clock::time_point startTime;
  std::chrono::steady_clock::time_point lastRealTime;
  };*/

class Simulator {
 public:
  Simulator();
  explicit Simulator(u32 _numThreads);
  virtual ~Simulator();

  // general info
  u32 threads() const;
  Time time() const;

  // events and event handling
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

 protected:
  // this is used for subclasses to generate thread specific data structures
  u32 threadId() const;

 private:
  // this defines the type of queue used for events
  typedef std::priority_queue<Event*, std::vector<Event*>,
                              EventComparator> EventQueue;

  // this defines a set of event queues for each executer thread
  struct alignas(CACHELINE_SIZE) QueueSet {
    // this is an initial padding incase the object is dynamically allocated,
    //  which does not adhere to the alignas specifier
    char padding0[CACHELINE_SIZE];

    // a multi-producer single-consumer event queue for lockfree staging
    MpScQueue oqueue;
    char padding1[CACHELINE_SIZE];

    // a time ordered event queue
    EventQueue pqueue;
    char padding2[CACHELINE_SIZE];
  };

  // this is the function that gets called per executer thread
  void executer(u32 _id);

  // info
  u32 numThreads_;
  SimState simState_;

  // per-thread queue sets
  QueueSet* queueSets_;

  // this is a logger for the entire simulation framework
  Logger* logger_;

  // models and debugging names
  std::unordered_map<std::string, Model*> models_;
  std::unordered_set<std::string> toBeDebugged_;
};

}  // namespace des

#endif  // DES_SIMULATOR_H_
