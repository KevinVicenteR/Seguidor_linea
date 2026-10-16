#include "Profiles.h"

namespace atlas {

// Los valores por defecto de RobotConfig corresponden al perfil Seguro
// (vuelta de reconocimiento).
RobotConfig safeProfile() { return RobotConfig(); }

// Perfil Rápido: intentos guiados por el mapa aprendido. Ajustado con
// simulation/tools/atlas_tune_learning sobre robots nominal, moderado y real.
RobotConfig fastProfile() {
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

void useNeuralNetworks(RobotConfig& config, bool enabled) {
  config.sensors.neuralPosition = enabled;
  config.control.neuralGains = enabled;
}

}  // namespace atlas
