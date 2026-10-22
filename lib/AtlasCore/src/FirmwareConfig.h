// Ajustes de despliegue compartidos por el firmware AVR y su simulación.
#pragma once
#include "Profiles.h"

namespace atlas {
constexpr bool kUseNeuralNetwork = true;

constexpr RobotConfig firmwareConfig(RobotConfig config, bool neural = kUseNeuralNetwork) {
  config.speed.basePwm = 60;
  config.speed.minPwm = 60;
  config.speed.maxPwm = 250;
  config.speed.stepPwm = 10;
  config.speed.runRampMs = 150;
  config.impeller.pwm = 235;
  config.impeller.runRampMs = 250;
  useNeuralNetworks(config, neural);
  return config;
}

constexpr RobotConfig firmwareSafeConfig(bool neural = kUseNeuralNetwork) {
  return firmwareConfig(safeProfile(), neural);
}

constexpr RobotConfig firmwareFastConfig(bool neural = kUseNeuralNetwork) {
  RobotConfig config = firmwareConfig(fastProfile(), neural);
  config.impeller.pwm = 235;
  return config;
}
}  // namespace atlas
