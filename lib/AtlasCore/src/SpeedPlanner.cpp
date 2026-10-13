#include "SpeedPlanner.h"

#include <math.h>

namespace atlas {

int16_t SpeedPlanner::limitFor(uint8_t curvature) const {
  const PlannerConfig& c = *config_;
  if (curvature <= c.straightCurvature) return c.maxPwm;
  const float limit = c.fullTurnPwm * sqrtf(255.0f / curvature);
  return limit > c.maxPwm ? c.maxPwm : static_cast<int16_t>(limit);
}

uint8_t SpeedPlanner::confidencePercent(const TrackMap& map) const {
  const PlannerConfig& c = *config_;
  if (map.lostPerMille() <= c.easyLostPerMille) return 100;
  const uint32_t penalty = static_cast<uint32_t>(map.lostPerMille() - c.easyLostPerMille) * c.slowdownPerLostPerMille;
  return penalty >= 100u - c.minConfidencePercent ? c.minConfidencePercent : static_cast<uint8_t>(100 - penalty);
}

int16_t SpeedPlanner::speedAt(const TrackMap& map, uint16_t bin) const {
  const PlannerConfig& c = *config_;
  if (!map.isValid() || bin >= map.length()) return c.offMapPwm;

  int32_t best = c.maxPwm;
  for (uint16_t ahead = 0; ahead <= c.lookaheadBins; ahead++) {
    const uint16_t j = bin + ahead;
    if (j >= map.length()) break;
    // Velocidad que permite llegar al tramo j sin pasarse de su límite.
    const int32_t allowed = limitFor(map.curvature(j)) + static_cast<int32_t>(c.brakePwmPerBin) * ahead;
    if (allowed < best) best = allowed;
  }
  return static_cast<int16_t>(best * confidencePercent(map) / 100);
}

}  // namespace atlas
