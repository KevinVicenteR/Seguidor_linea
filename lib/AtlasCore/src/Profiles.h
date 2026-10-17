/*
  Profiles.h - Perfiles de manejo.

  - Seguro: ajustado para no fallar aunque el robot real sea más lento, tenga
    más inercia o más retraso que el modelo (optimización robusta).
  - Rápido: más agresivo; para cuando el robot ya se probó estable.

  Ambos se obtuvieron con simulation/tools/atlas_tune (--robusto / --rapido).
*/
#pragma once

#include "Config.h"
#include "NeuralDerivativeWeights.h"

namespace atlas {

// constexpr (C++14): el firmware las calcula al compilar en vez de armar las
// configuraciones al arrancar, lo que ahorra ~2 KB de flash.
constexpr RobotConfig safeProfile() { return RobotConfig(); }

constexpr RobotConfig fastProfile() {
  RobotConfig config;
  // Barrido de 32,406 competencias simuladas contra la oscilación (antes 0.008 / 1.0).
  config.control.kp = 0.0068f;
  config.control.kd = 0.99f;
  config.control.derivativeFilterShift = 1;
  config.control.curveSlowdownPercent = 0;  // el mapa ya decide la velocidad de cada tramo
  config.lostLine.outerMotorPwm = 201;

  config.planner.maxPwm = 255;
  config.planner.fullTurnPwm = 255;  // con turbina 200 en las vueltas rápidas
  config.planner.straightCurvature = 26;
  config.planner.brakePwmPerBin = 5;
  config.planner.lookaheadBins = 18;
  return config;
}

// Redes neuronales (NeuralPositionWeights.h): true = activas, false = control
// clásico idéntico al anterior.
// La red jerárquica de control quedó neutra tras entrenarla (no superó a la
// referencia): se apaga para no gastar tiempo de cálculo. La red de la
// derivada solo se activa cuando ya fue entrenada.
constexpr void useNeuralNetworks(RobotConfig& config, bool enabled) {
  config.sensors.neuralPosition = enabled;
  config.control.neuralGains = false;
  config.control.neuralDerivative = enabled && neural::kDerivativeTrained;
}

}  // namespace atlas
