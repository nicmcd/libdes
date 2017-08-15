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
#include <utility>
#include <vector>

#include "des/cacheline_util.h"
#include "des/Event.h"
#include "des/MpScQueue.h"
#include "des/Time.h"

namespace des {

class ActiveComponent;
class Mapper;
class Logger;
class Component;
class Observer;

class Simulator {
 public:
  // construction and desctruction
  Simulator();
  explicit Simulator(u32 _numExecuters);
  virtual ~Simulator();

  // general info
  u32 executers() const;
  Time time() const;

  // component->executer mapping
  void setMapper(Mapper* _mapper);
  Mapper* getMapper() const;

  // logging
  void setLogger(Logger* _logger);
  Logger* getLogger() const;

  // observing
  void addObserver(Observer* _observer);
  void setObservingIntervel(f64 _interval);
  void setObservingPower(u64 _expPow2Events);

  // components and debugging
  void addComponent(Component* _component);
  void mapComponent(ActiveComponent* _component);
  Component* getComponent(const std::string& _fullname) const;
  void removeComponent(const std::string& _fullname);
  u64 numComponents() const;
  void addDebugName(const std::string& _fullname);
  void debugCheck();

  // clocks
  u32 addClock(Tick _period, Tick _phase);
  Tick clockPeriod(u32 _clockId) const;
  Tick clockPhase(u32 _clockId) const;
  u64 cycle(u32 _clockId) const;
  u64 cycles(u32 _clockId, Time _time) const;
  Time futureCycle(u32 _clockId, u32 _cycles) const;
  Time futureCycle(u32 _clockId, u32 _cycles, Epsilon _epsilon) const;

  // events and event handling
  void addEvent(Event* _event);
  void initialize();
  void simulate();

 protected:
  // this is used for subclasses to generate executer specific data structures
  u32 executerId() const;

 private:
  // this defines the type of queue used for events
  typedef std::priority_queue<Event*, std::vector<Event*>,
                              EventComparator> EventQueue;

  // this defines a set of event queues for each executer
  //  these are cache line aligned and padded individually
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

  // these variables are read/write shared variables for the executers
  //  these are cache line aligned and padded individually
  struct alignas(CACHELINE_SIZE) State {
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
  };

  // these variables are read/write shared variables for the purpose of stats
  //  these are partially cache line aligned and padded individually
  struct alignas(CACHELINE_SIZE) Stats {
    // this is an initial padding incase the object is dynamically allocated,
    //  which does not adhere to the alignas specifier
    char padding0[CACHELINE_SIZE];

    // this variable counts total number of executed events
    std::atomic<u64> eventCount;
    char padding2[CLPAD(sizeof(eventCount))];

    // this variable counts total number of executed time steps
    u64 timeStepCount;
    char padding3[CLPAD(sizeof(timeStepCount))];

    // this variable tracks the start time of simulation
    std::chrono::steady_clock::time_point startRealTime;

    // this variable tracks the last time stats were printed
    std::chrono::steady_clock::time_point lastRealTime;

    // this variable tracks the last amount of events reported
    u64 lastEventCount;

    // this variable tracks the last amount of timeSteps reported
    u64 lastTimeStepCount;

    // this variable tracks the last tick reported
    u64 lastTick;
  };

  // this is the function that gets called per executer thread
  void executer(u32 _id);

  // get the initial time step to be simulated by the executers
  TimeStep initialTimeStep();

  // this initializes the barrier
  // Args:
  //  _firstTimeStep - the first time step to be simulated
  void barrierInit(TimeStep _firstTimeStep);

  // this is the barrier function each executer thread calls on each time step
  // Args:
  //  _id - the ID of this executer
  //  _minTimeStep - minimum timestep desired by this executer
  //  _executed - number of events executed by this executer
  // Return:
  //  the next TimeStep to be executed
  TimeStep barrier(u32 _id, TimeStep _minTimeStep, u64 _executed);

  // info
  const u32 numExecuters_;
  State state_;
  Stats stats_;

  // per-executer queue sets
  QueueSet* queueSets_;

  // logger
  Logger* logger_;

  // these are for observing simulator performance
  std::vector<Observer*> observers_;
  f64 observingInterval_;
  u64 observingMask_;

  // components and debugging names
  std::unordered_map<std::string, Component*> components_;
  std::unordered_set<std::string> toBeDebugged_;
  bool initialized_;

  // component to executer mapping
  Mapper* mapper_;

  // clocks
  std::vector<std::pair<Tick, Tick> > clocks_;
};

}  // namespace des

#endif  // DES_SIMULATOR_H_
