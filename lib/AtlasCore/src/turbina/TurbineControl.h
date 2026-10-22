#pragma once
#include "../parametros/ImpellerConfig.h"
#include "../Ramp.h"

namespace atlas {
// Una misma lógica de rampa para PROGRAM/READY y para la carrera.
class TurbineControl {
 public:
  int16_t prepare(bool ready, uint32_t now, const ImpellerConfig& config) {
    if (!config.spinWithReady || !ready) {
      stop();
      return 0;
    }
    if (!active_) {
      ramp_.start(now, config.startRampMs, 0, config.pwm);
      active_ = true;
    }
    return pwmAt(now);
  }
  void startRun(uint32_t now, const ImpellerConfig& config, int16_t initialPwm) {
    ramp_.start(now, config.runRampMs, initialPwm, config.pwm);
    active_ = true;
  }
  int16_t pwmAt(uint32_t now) const { return active_ ? ramp_.valueAt(now) : 0; }
  bool atSpeed(uint32_t now) const { return active_ && ramp_.isFinished(now); }
  void stop() { active_ = false; }

 private:
  LinearRamp ramp_;
  bool active_ = false;
};
}  // namespace atlas
