// Segunda ronda: perfil Rápido; el mapa limita la velocidad de cada tramo.
#pragma once
#include "../parametros/FirmwareParameters.h"

namespace atlas {
constexpr int16_t kSegundaRondaMotorMaxPwm = 60;
constexpr int16_t kSegundaRondaCurvaPwm = 60;
constexpr int16_t kSegundaRondaTurbinaPwm = 235;

constexpr RobotConfig segundaRondaConfig(bool neural = kUseNeuralNetwork) {
  RobotConfig config = firmwareConfig(fastProfile(), neural);
  config.speed.basePwm = config.speed.minPwm;
  config.planner.maxPwm = kSegundaRondaMotorMaxPwm;
  config.planner.fullTurnPwm = kSegundaRondaCurvaPwm;
  config.impeller.pwm = kSegundaRondaTurbinaPwm;
  return config;
}
}  // namespace atlas
