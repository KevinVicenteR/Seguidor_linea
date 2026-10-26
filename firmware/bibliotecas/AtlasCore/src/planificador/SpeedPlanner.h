#pragma once

#include <stdint.h>

#include "Config.h"
#include "TrackMap.h"

namespace atlas {

class SpeedPlanner {
 public:
  explicit SpeedPlanner(const PlannerConfig& config) : config_(&config) {}
  void configure(const PlannerConfig& config) { config_ = &config; }

  int16_t limitFor(uint8_t curvature, uint8_t curveSpeedPercent = 100) const;
  uint8_t confidencePercent(const TrackMap& map) const;
  int16_t speedAt(const TrackMap& map, uint16_t bin) const;

 private:
  const PlannerConfig* config_;
};

}
