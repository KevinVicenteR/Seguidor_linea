// Primera ronda: reconocimiento, perfil Seguro y grabación del mapa.
#pragma once
#include "../parametros/FirmwareParameters.h"

namespace atlas {
constexpr int16_t kPrimeraRondaMotorPwm = 60;
constexpr int16_t kPrimeraRondaTurbinaPwm = 255;

constexpr RobotConfig primeraRondaConfig(bool neural = kUseNeuralNetwork) {
  RobotConfig config = firmwareConfig(safeProfile(), neural);
  config.speed.basePwm = kPrimeraRondaMotorPwm;
  config.impeller.pwm = kPrimeraRondaTurbinaPwm;
  return config;
}
}  // namespace atlas
