#include "SpeedGovernor.h"

namespace atlas {

void SpeedGovernor::reset(uint32_t nowMs) {
  speedPercent_ = 100;
  lastChangeMs_ = nowMs;
}

void SpeedGovernor::penalize(uint8_t percent, uint32_t nowMs) {
  if (!config_->speedGovernor) return;
  speedPercent_ = speedPercent_ > config_->minSpeedPercent + percent ? speedPercent_ - percent : config_->minSpeedPercent;
  lastChangeMs_ = nowMs;
}

void SpeedGovernor::recover(uint32_t nowMs) {
  if (speedPercent_ >= 100 || nowMs - lastChangeMs_ < config_->speedRecoverMs) return;
  speedPercent_++;
  lastChangeMs_ = nowMs;
}

}
