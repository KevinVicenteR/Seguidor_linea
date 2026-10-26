#pragma once
#include "Profiles.h"

namespace atlas {
constexpr bool kUseNeuralNetwork = true;

constexpr RobotConfig firmwareConfig(RobotConfig config, bool neural = kUseNeuralNetwork) {
  config.speed.basePwm = 130;
  config.speed.minPwm = 60;
  config.speed.maxPwm = 250;
  config.speed.stepPwm = 10;
  config.speed.runRampMs = 150;
  config.speed.maxAccelPwmPerS = 400;
  config.speed.startKickMs = 0;
  config.impeller.pwm = 0;
  config.impeller.spinWithReady = false;
  config.lostLine.outerMotorPwm = 110;
  config.corner.speedPercent = 50;
  config.planner.brakePwmPerBin = 7;
  useNeuralNetworks(config, neural);
  return config;
}

constexpr RobotConfig firmwareSafeConfig(bool neural = kUseNeuralNetwork) {
  return firmwareConfig(safeProfile(), neural);
}

constexpr RobotConfig firmwareFastConfig(bool neural = kUseNeuralNetwork) {
  RobotConfig config = firmwareConfig(fastProfile(), neural);
  config.planner.maxPwm = 225;
  config.planner.fullTurnPwm = 140;
  return config;
}
}
