#include "TrackMap.h"

namespace atlas {

void TrackMap::clear() {
  curveSpeedPercent_ = 100;
  length_ = 0;
  gapCount_ = 0;
  valid_ = false;
}

void TrackMap::startRecording() {
  clear();
  for (uint16_t i = 0; i < kMaxMapBins; i++) bins_[i] = 0;
  openBin_ = 0;
  binSum_ = 0;
  binSamples_ = 0;
  binLost_ = false;
}

void TrackMap::recordCurvature(uint16_t bin, uint8_t intensity, bool lineLost) {
  if (bin != openBin_) {
    closeBin();
    openBin_ = bin;
  }
  binSum_ += intensity;
  binSamples_++;
  binLost_ = binLost_ || lineLost;
}

void TrackMap::closeBin() {
  if (openBin_ < kMaxMapBins && binSamples_ > 0) {
    bins_[openBin_] = binLost_ ? 255 : static_cast<uint8_t>(binSum_ / binSamples_);
  }
  binSum_ = 0;
  binSamples_ = 0;
  binLost_ = false;
}

void TrackMap::recordGap(uint16_t bin) {
  if (gapCount_ == kMaxMapGaps) return;
  if (gapCount_ > 0 && bin <= gaps_[gapCount_ - 1] + 2) return;  // el mismo hueco
  gaps_[gapCount_++] = bin;
}

void TrackMap::finishRecording(uint16_t lastBin, uint16_t minimumBins, uint16_t lostPerMille) {
  closeBin();
  lostPerMille_ = lostPerMille;
  length_ = lastBin + 1 < kMaxMapBins ? lastBin + 1 : kMaxMapBins;
  valid_ = length_ >= minimumBins;
}

int32_t TrackMap::nearestGap(uint16_t bin, uint16_t window) const {
  int32_t best = -1;
  uint16_t bestDistance = window + 1;
  for (uint8_t i = 0; i < gapCount_; i++) {
    const uint16_t distance = gaps_[i] > bin ? gaps_[i] - bin : bin - gaps_[i];
    if (distance < bestDistance) {
      bestDistance = distance;
      best = gaps_[i];
    }
  }
  return best;
}

void TrackMap::markDangerZone(uint16_t bin, uint8_t before, uint8_t after, uint8_t boost) {
  const uint16_t first = bin > before ? bin - before : 0;
  for (uint16_t i = first; i <= bin + after && i < length_; i++) {
    const uint16_t raised = static_cast<uint16_t>(bins_[i]) + boost;
    bins_[i] = raised > 255 ? 255 : static_cast<uint8_t>(raised);
  }
}

void TrackMap::reduceCurveSpeed(uint8_t step, uint8_t minimum) {
  curveSpeedPercent_ = curveSpeedPercent_ > minimum + step ? curveSpeedPercent_ - step : minimum;
}

void TrackMap::restore(uint16_t length, uint8_t gapCount, uint16_t lostPerMille, uint8_t curveSpeedPercent) {
  lostPerMille_ = lostPerMille;
  curveSpeedPercent_ = curveSpeedPercent > 100 ? 100 : curveSpeedPercent;
  length_ = length > kMaxMapBins ? kMaxMapBins : length;
  gapCount_ = gapCount > kMaxMapGaps ? kMaxMapGaps : gapCount;
  valid_ = length_ > 0;
}

}  // namespace atlas
