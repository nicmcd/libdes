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
#include "des/Logger.h"

#include "des/Simulator.h"

namespace des {

Logger::Logger(des::Simulator* _simulator, const std::string& _name,
               const Model* _parent)
    : des::Model(_simulator, _name, _parent) {}

Logger::~Logger() {}

void Logger::log(char* _message) {
  des::EventHandler handler = static_cast<des::EventHandler>(
      &Logger::logHandler);
  simulator()->addEvent(new LogEvent(this, handler,
                                     simulator()->time(),
                                     simulator()->epsilon() + 1,
                                     _message));
}

Logger::LogEvent::LogEvent(des::Model* _model, des::EventHandler _handler,
                           u64 _time, u8 _epsilon, char* _message)
    : des::Event(_model, _handler, _time, _epsilon), message(_message) {}

void Logger::logHandler(des::Event* _event) {
  LogEvent* evt = reinterpret_cast<LogEvent*>(_event);
  printf("%s", evt->message);
  delete[] evt->message;
  delete evt;
}

}  // namespace des
