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
#include "des/util/BasicObserver.h"

#include <cassert>

static const u64 STATS_SIZE = 1024;

static const u64 dayFactor = 24 * 60 * 60 * 1000;
static const u64 hourFactor = 60 * 60 * 1000;
static const u64 minuteFactor = 60 * 1000;
static const u64 secondFactor = 1000;

namespace des {

BasicObserver::BasicObserver(Logger* _logger, bool _logSummary)
    : logger_(_logger), logSummary_(_logSummary),
      statsString_(nullptr) {
  statsString_ = new char[STATS_SIZE];
}

BasicObserver::~BasicObserver() {
  delete[] statsString_;
}

void BasicObserver::progressStatistics(
    const Observer::ProgressStatistics& _progressStats) {
  f64 totalRealTime = _progressStats.seconds;

  u64 milliseconds = static_cast<u32>(totalRealTime * 1000);
  u64 days = milliseconds / dayFactor;
  milliseconds %= dayFactor;
  u64 hours = milliseconds / hourFactor;
  milliseconds %= hourFactor;
  u64 minutes = milliseconds / minuteFactor;
  milliseconds %= minuteFactor;
  u64 seconds = milliseconds / secondFactor;
  milliseconds %= secondFactor;

  s32 r = snprintf(
      statsString_, STATS_SIZE,
      "%lu:%02lu:%02lu:%02lu : "
      "%lu events : %lu ticks : %.0f events/sec : "
      "%.0f ticks/sec : %.0f steps/sec\n",
      days, hours, minutes, seconds,
      _progressStats.eventCount,
      _progressStats.ticks,
      _progressStats.eventsPerSecond,
      _progressStats.ticksPerSecond,
      _progressStats.stepsPerSecond);
  assert(r > 0 && r < (s32)STATS_SIZE);
  logger_->log(statsString_, r);
}

void BasicObserver::summaryStatistics(
    const Observer::SummaryStatistics& _summaryStats) {
  if (logSummary_) {
    f64 eventsPerSecond = _summaryStats.eventCount / _summaryStats.seconds;
    f64 eventsPerTick = _summaryStats.eventCount / _summaryStats.ticks;
    f64 ticksPerSecond = _summaryStats.ticks / _summaryStats.seconds;
    f64 stepsPerSecond = _summaryStats.timeSteps / _summaryStats.seconds;

    s32 r = snprintf(
        statsString_, STATS_SIZE,
        "\n"
        "Total event count:     %lu\n"
        "Total time steps:      %lu\n"
        "Total seconds:         %.3f\n"
        "\n"
        "Events per second:     %.3f\n"
        "Events per sim tick:   %.3f\n"
        "Sim ticks per second:  %.3f\n"
        "Time steps per second: %.3f\n"
        "\n",
        _summaryStats.eventCount,
        _summaryStats.timeSteps,
        _summaryStats.seconds,
        eventsPerSecond,
        eventsPerTick,
        ticksPerSecond,
        stepsPerSecond);
    assert(r > 0 && r < (s32)STATS_SIZE);
    logger_->log(statsString_, r);
  }
}

}  // namespace des
