#pragma once

#include "Config.h"
#include "NeuralDerivativeWeights.h"

namespace atlas {

constexpr RobotConfig safeProfile() { return RobotConfig(); }

constexpr RobotConfig fastProfile() {
  RobotConfig config;
  config.control.kp = 0.0068f;
  config.control.kd = 0.99f;
  config.control.derivativeFilterShift = 1;
  config.control.curveSlowdownPercent = 0;
  config.lostLine.outerMotorPwm = 201;

  config.planner.maxPwm = 255;
  config.planner.fullTurnPwm = 255;
  config.planner.straightCurvature = 26;
  config.planner.brakePwmPerBin = 5;
  config.planner.lookaheadBins = 18;
  return config;
}

constexpr uint8_t kDeployedDerivativeBlend = 0;

constexpr void useNeuralNetworks(RobotConfig& config, bool enabled) {
  config.sensors.neuralPosition = enabled;
  config.control.neuralGains = false;
  config.control.neuralDerivative = enabled && neural::kDerivativeTrained && kDeployedDerivativeBlend > 0;
  config.control.derivativeBlend = kDeployedDerivativeBlend;
}

}
