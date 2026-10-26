// Ajustes de despliegue compartidos por el firmware AVR y su simulación.
// Rama sin turbina: el robot corre sin impeller, así que el agarre lateral es
// solo μ·g (~1 g). Los valores salen de un barrido en simulación sobre las
// pistas obligatorias con robots nominal, moderado y real (0 fallas en 792
// carreras; también 0 fallas con llantas al 90 % de agarre).
#pragma once
#include "Profiles.h"

namespace atlas {
constexpr bool kUseNeuralNetwork = true;

constexpr RobotConfig firmwareConfig(RobotConfig config, bool neural = kUseNeuralNetwork) {
  config.speed.basePwm = 130;  // con turbina 170: sin succión derrapa en el reconocimiento
  config.speed.minPwm = 60;
  config.speed.maxPwm = 250;
  config.speed.stepPwm = 10;
  config.speed.runRampMs = 150;
  // Llantas de mucho agarre sin turbina: acelerar de golpe hace caballito.
  // 400 PWM/s = de 0 a 130 en ~0.33 s (en simulación cuesta ~0.1 s por vuelta).
  // Si aún levanta el frente, bájalo (250 cuesta ~0.4 s por vuelta).
  config.speed.maxAccelPwmPerS = 400;
  config.speed.startKickMs = 0;  // el impulso de arranque era para la succión
  config.impeller.pwm = 0;  // sin turbina
  config.impeller.spinWithReady = false;
  config.lostLine.outerMotorPwm = 110;  // buscar la línea sin derrapar
  config.corner.speedPercent = 50;      // frenar más en esquinas cerradas
  config.planner.brakePwmPerBin = 7;
  useNeuralNetworks(config, neural);
  return config;
}

constexpr RobotConfig firmwareSafeConfig(bool neural = kUseNeuralNetwork) {
  return firmwareConfig(safeProfile(), neural);
}

constexpr RobotConfig firmwareFastConfig(bool neural = kUseNeuralNetwork) {
  RobotConfig config = firmwareConfig(fastProfile(), neural);
  config.planner.maxPwm = 225;       // rectas conocidas
  config.planner.fullTurnPwm = 140;  // con turbina 255
  return config;
}
}  // namespace atlas
