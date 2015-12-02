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

#include <cassert>

namespace des {

Logger::Logger()
    : Logger("-") {}

Logger::Logger(const std::string& _filename) {
  if (_filename == "-") {
    close_ = false;
    compress_ = false;
    regFile_ = stdout;
  } else if (_filename == "+") {
    close_ = false;
    compress_ = false;
    regFile_ = stderr;
  } else if (_filename.size() >= 3 &&
             _filename.substr(_filename.size() - 3) == ".gz") {
    close_ = true;
    compress_ = true;
    gzFile_ = gzopen(_filename.c_str(), "wb");
    if (!gzFile_) {
      fprintf(stderr, "couldn't open gz file: %s\n", _filename.c_str());
      exit(-1);
    }
  } else {
    close_ = true;
    compress_ = false;
    regFile_ = fopen(_filename.c_str(), "w");
    if (!regFile_) {
      fprintf(stderr, "couldn't open regular file: %s\n", _filename.c_str());
      exit(-1);
    }
  }
}

Logger::~Logger() {
  if (close_) {
    if (compress_) {
      gzclose(gzFile_);
    } else {
      fclose(regFile_);
    }
  }
}

void Logger::log(const char* _message, u64 _len) {
  lock_.lock();
  if (compress_) {
    assert(gzwrite(gzFile_, _message, _len) == (s64)_len);
  } else {
    assert(fwrite(_message, sizeof(char), _len, regFile_) == _len);
  }
  lock_.unlock();
}

}  // namespace des
