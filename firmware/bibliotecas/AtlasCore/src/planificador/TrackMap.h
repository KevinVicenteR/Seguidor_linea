#pragma once

#include <stdint.h>

namespace atlas {

constexpr uint16_t kMaxMapBins = 320;
constexpr uint8_t kMaxMapGaps = 16;

class TrackMap {
 public:
  void clear();
  void startRecording();
  void recordCurvature(uint16_t bin, uint8_t intensity, bool lineLost);
  void recordGap(uint16_t bin);
  void finishRecording(uint16_t lastBin, uint16_t minimumBins, uint16_t lostPerMille);

  bool isValid() const { return valid_; }
  uint16_t length() const { return length_; }
  uint8_t curvature(uint16_t bin) const { return bin < kMaxMapBins ? bins_[bin] : 255; }
  uint8_t gapCount() const { return gapCount_; }
  uint16_t lostPerMille() const { return lostPerMille_; }
  uint16_t gap(uint8_t index) const { return gaps_[index]; }
  int32_t nearestGap(uint16_t bin, uint16_t window) const;

  uint8_t* rawBins() { return bins_; }
  uint16_t* rawGaps() { return gaps_; }
  void restore(uint16_t length, uint8_t gapCount, uint16_t lostPerMille, uint8_t curveSpeedPercent);
  uint8_t curveSpeedPercent() const { return curveSpeedPercent_; }
  void reduceCurveSpeed(uint8_t step, uint8_t minimum);
  void markDangerZone(uint16_t bin, uint8_t before, uint8_t after, uint8_t boost);
  void invalidate() { valid_ = false; }

 private:
  uint8_t bins_[kMaxMapBins] = {};
  uint16_t gaps_[kMaxMapGaps] = {};
  uint16_t length_ = 0;
  uint8_t gapCount_ = 0;
  uint16_t lostPerMille_ = 0;
  uint8_t curveSpeedPercent_ = 100;
  bool valid_ = false;

  void closeBin();
  uint16_t openBin_ = 0;
  uint32_t binSum_ = 0;
  uint16_t binSamples_ = 0;
  bool binLost_ = false;
};

}
