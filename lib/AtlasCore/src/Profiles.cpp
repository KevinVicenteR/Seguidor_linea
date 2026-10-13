#include "Profiles.h"

namespace atlas {

// Los valores por defecto de RobotConfig corresponden al perfil Seguro
// (vuelta de reconocimiento).
RobotConfig safeProfile() { return RobotConfig(); }

// Perfil Rápido: intentos guiados por el mapa aprendido. Ajustado con
// simulation/tools/atlas_tune_learning sobre robots nominal, moderado y real.
RobotConfig fastProfile() {
  RobotConfig config;
  config.control.kp = 0.010f;
  config.control.kd = 1.6f;  // bajada de 1.75: el robot real vibraba
  config.control.derivativeFilterShift = 1;
  config.control.curveSlowdownPercent = 0;  // el mapa ya decide la velocidad de cada tramo
  config.lostLine.outerMotorPwm = 201;

  config.planner.maxPwm = 239;
  config.planner.fullTurnPwm = 133;
  config.planner.straightCurvature = 26;
  config.planner.brakePwmPerBin = 5;
  config.planner.lookaheadBins = 18;
  return config;
}

}  // namespace atlas
