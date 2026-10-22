// Tercera ronda y posteriores: ajustes independientes, con el mapa corregido.
#pragma once
#include "../parametros/FirmwareParameters.h"

namespace atlas {
constexpr int16_t kTerceraRondaMotorMaxPwm = 60;
constexpr int16_t kTerceraRondaCurvaPwm = 60;
constexpr int16_t kTerceraRondaTurbinaPwm = 235;

constexpr RobotConfig terceraRondaConfig(bool neural = kUseNeuralNetwork) {
  RobotConfig config = firmwareConfig(fastProfile(), neural);
  config.speed.basePwm = config.speed.minPwm;
  config.planner.maxPwm = kTerceraRondaMotorMaxPwm;
  config.planner.fullTurnPwm = kTerceraRondaCurvaPwm;
  config.impeller.pwm = kTerceraRondaTurbinaPwm;
  return config;
}
}  // namespace atlas
