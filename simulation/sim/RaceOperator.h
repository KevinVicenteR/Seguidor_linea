/*
  RaceOperator.h - Operador humano simulado. Ejecuta exactamente la
  secuencia del manual de ATLAS sobre el firmware real:

    1. Selecciona color de línea (SW1 blanca / SW2 negra).
    2. Pasa la barra lentamente sobre la línea y termina la calibración.
    3. Coloca el robot en la salida y ajusta la velocidad con SW1/SW2.
    4. Activa READY (pre-giro del impeller) y luego GO.
    5. Cuando se completan las vueltas (o el robot falla) quita GO.
*/
#pragma once

#include "Config.h"
#include "RaceMonitor.h"
#include "RobotModel.h"
#include "SimHardware.h"

namespace sim {

struct OperatorPlan {
  atlas::LineColor lineColor = atlas::LineColor::White;
  int speedPresses = 0;  // positivo = SW2 (subir), negativo = SW1 (bajar)
  Pose startPose;        // pose del robot con la barra centrada en la línea
  double raceTimeoutS = 90.0;
};

class RaceOperator {
 public:
  RaceOperator(SimOperatorPanel& panel, RobotModel& robot, RaceMonitor& monitor, const OperatorPlan& plan);

  void update(double timeS);
  bool hasReleasedGo() const { return phase_ == Phase::Done; }

 private:
  enum class Phase { WaitBoot, PressColor, Sweep, PressEndCalibration, WaitConfirm, AdjustSpeed, Ready, Race, Done };

  static constexpr double kBootS = 1.0;
  static constexpr double kPressS = 0.12;
  static constexpr double kSweepS = 2.4;
  static constexpr double kSweepAmplitude = 0.09;
  static constexpr double kConfirmWaitS = 2.0;
  static constexpr double kSpeedPressPeriodS = 0.8;
  static constexpr double kReadyS = 1.3;

  void enter(Phase phase, double timeS);
  void holdAtLateralOffset(double offset);
  void pressColorButton(bool pressed);
  void pressSpeedButton(bool pressed);

  SimOperatorPanel& panel_;
  RobotModel& robot_;
  RaceMonitor& monitor_;
  OperatorPlan plan_;
  Phase phase_ = Phase::WaitBoot;
  double phaseStart_ = 0.0;
  int pressesLeft_;
};

}  // namespace sim
