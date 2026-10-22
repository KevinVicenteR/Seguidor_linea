// Parámetros comunes. Las velocidades de cada ronda viven en rondas/.
#pragma once
#include "Profiles.h"

namespace atlas {
constexpr bool kUseNeuralNetwork = true;

constexpr RobotConfig firmwareConfig(RobotConfig config, bool neural = kUseNeuralNetwork) {
  config.speed.minPwm = 60;
  config.speed.startKickPwm = config.speed.minPwm;
  config.speed.maxPwm = 250;
  config.speed.stepPwm = 10;
  config.speed.runRampMs = 150;
  config.impeller = ImpellerConfig();
  useNeuralNetworks(config, neural);
  return config;
}
}  // namespace atlas
