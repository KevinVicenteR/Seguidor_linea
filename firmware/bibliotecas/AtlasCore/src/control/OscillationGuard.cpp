#include "OscillationGuard.h"

namespace atlas {

constexpr uint8_t OscillationGuard::kMaxCrossings;

void OscillationGuard::reset(uint32_t nowMs) {
  gainPercent_ = 100;
  lastSide_ = 0;
  crossingCount_ = 0;
  lastChangeMs_ = nowMs;
}

bool OscillationGuard::update(int32_t error, uint32_t nowMs) {
  if (!config_->autoDamping) return false;

  const int8_t side = error > config_->oscillationAmplitude ? 1 : (error < -config_->oscillationAmplitude ? -1 : 0);
  bool oscillating = false;
  if (side != 0 && lastSide_ != 0 && side != lastSide_) oscillating = registerCrossing(nowMs);
  if (side != 0) lastSide_ = side;

  if (oscillating) {
    const uint8_t step = config_->gainStepDownPercent;
    gainPercent_ = gainPercent_ > config_->minGainPercent + step ? gainPercent_ - step : config_->minGainPercent;
    crossingCount_ = 0;
    lastChangeMs_ = nowMs;
  } else {
    recover(nowMs);
  }
  return oscillating;
}

bool OscillationGuard::registerCrossing(uint32_t nowMs) {
  if (crossingCount_ == kMaxCrossings) {
    for (uint8_t i = 1; i < kMaxCrossings; i++) crossings_[i - 1] = crossings_[i];
    crossingCount_--;
  }
  crossings_[crossingCount_++] = nowMs;

  uint8_t recent = 0;
  for (uint8_t i = 0; i < crossingCount_; i++) {
    if (nowMs - crossings_[i] <= config_->oscillationWindowMs) recent++;
  }
  return recent >= config_->oscillationCrossings;
}

void OscillationGuard::recover(uint32_t nowMs) {
  if (gainPercent_ >= 100 || nowMs - lastChangeMs_ < config_->gainRecoverMs) return;
  gainPercent_++;
  lastChangeMs_ = nowMs;
}

}
