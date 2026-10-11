/*
  RobotModel.h - Modelo físico simplificado del ATLAS Rev. 1.4.

  - Tracción diferencial con motores de primer orden (constante de tiempo).
  - Límite de adherencia: la aceleración (longitudinal y lateral) no puede
    superar μ·(g + F_succión/m). El impeller aumenta la fuerza normal; si se
    pide más giro del que permite la adherencia, el robot derrapa (abre la
    trayectoria), que es el modo de falla real a alta velocidad.
  - Geometría: barra de 16 sensores al frente del eje de las ruedas.
*/
#pragma once

#include "Geometry.h"

namespace sim {

struct RobotParams {
  // Geometría (manual: 17 x 24.5 cm)
  double wheelBase = 0.120;
  double sensorBarAhead = 0.180;  // distancia del eje de ruedas a la barra
  double sensorPitch = 0.009;     // separación entre sensores

  // Motores 1023 coreless con 2S
  double maxWheelSpeed = 3.2;  // m/s con PWM 255
  double motorTimeConstant = 0.035;
  double coastTimeConstant = 0.300;  // drivers deshabilitados

  // Adherencia y succión
  double mass = 0.115;  // kg con batería
  double tireFriction = 1.0;
  double impellerDownforce = 2.5;  // N con PWM 200
  double impellerReferencePwm = 200;
  double impellerTimeConstant = 0.120;
};

struct DriveInputs {
  int leftPwm = 0;
  int rightPwm = 0;
  int impellerPwm = 0;
  bool driversEnabled = false;
};

class RobotModel {
 public:
  static constexpr double kGravity = 9.81;

  explicit RobotModel(const RobotParams& params) : params_(params) {}

  void place(const Pose& pose);
  void setHeld(bool held) { held_ = held; }
  bool isHeld() const { return held_; }

  void step(double dt, const DriveInputs& inputs);

  const Pose& pose() const { return pose_; }
  double speed() const { return speed_; }
  double yawRate() const { return yawRate_; }
  bool isSliding() const { return sliding_; }
  double gripLimit() const;

  Vec2 sensorPosition(int index, int sensorCount) const;
  Vec2 sensorBarCenter() const { return pose_.toWorld(params_.sensorBarAhead, 0.0); }
  const RobotParams& params() const { return params_; }

 private:
  double wheelTarget(int pwm, bool enabled) const;

  RobotParams params_;
  Pose pose_;
  bool held_ = true;
  double leftWheel_ = 0.0;
  double rightWheel_ = 0.0;
  double speed_ = 0.0;
  double yawRate_ = 0.0;
  double downforce_ = 0.0;
  bool sliding_ = false;
};

}  // namespace sim
