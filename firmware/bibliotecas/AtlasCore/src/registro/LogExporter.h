#pragma once

#include <stdint.h>

#include "RunLog.h"
#include "TrackMap.h"

namespace atlas {

class ITextSink {
 public:
  virtual ~ITextSink() {}
  virtual void text(const char* value) = 0;
  virtual void number(int32_t value) = 0;
  virtual void endLine() = 0;
};

void exportRunLog(ITextSink& out, RunLog& log, const TrackMap& map);

}
