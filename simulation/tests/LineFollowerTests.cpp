#include "Fakes.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "TestFramework.h"

using namespace atlas;

namespace {

struct Rig {
  test::FakeSensorBus bus;
  test::FakeDrivetrain drivetrain;
  RobotConfig config;
  LineSensorArray sensors{bus, config.sensors};
  LineFollower follower{sensors, drivetrain, config};

  Rig() {
    sensors.resetCalibration();
    bus.fill(30);
    sensors.collectCalibrationSamples();
    bus.fill(220);
    sensors.collectCalibrationSamples();
    sensors.finishCalibration();
  }
};

// Desplaza la línea un sensor por ciclo de control, como ocurre en la pista.
void slideLineTo(Rig& rig, int from, int to, uint32_t& nowMs) {
  const int step = to > from ? 1 : -1;
  for (int first = from; first != to + step; first += step) {
    rig.bus.lineAt(first, first + 1);
    rig.follower.update(nowMs++);
  }
}

}  // namespace

TEST("start habilita drivers y arranca con rampa de velocidad") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(200, 0);
  EXPECT_TRUE(rig.drivetrain.enabled);
  rig.follower.update(0);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
  rig.follower.update(75);  // mitad de la rampa de 150 ms
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 100);
  rig.follower.update(1000);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 200);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, 200);
}

TEST("Línea a la derecha: la rueda izquierda acelera para girar a la derecha") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  uint32_t now = 1000;
  slideLineTo(rig, 7, 10, now);
  EXPECT_TRUE(rig.drivetrain.leftMotor.pwm > rig.drivetrain.rightMotor.pwm);
}

TEST("Salidas de rueda saturadas a los límites del firmware") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(250, 0);
  uint32_t now = 1000;
  slideLineTo(rig, 7, 13, now);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, rig.config.control.maxForwardPwm);
  EXPECT_TRUE(rig.drivetrain.rightMotor.pwm >= rig.config.control.maxBackwardPwm);
}

TEST("Línea perdida: gira hacia el último lado con freno breve de la rueda interior") {
  Rig rig;
  rig.bus.lineAt(1, 2);  // la línea se va por la izquierda
  rig.follower.start(150, 0);
  rig.follower.update(1000);
  rig.bus.fill(30);
  rig.follower.update(1010);  // 10 ms sin línea
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, rig.config.lostLine.innerMotorPwm);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, rig.config.lostLine.outerMotorPwm);
  rig.follower.update(1100);  // pasó el tiempo de freno (35 ms)
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, rig.config.lostLine.outerMotorPwm);
}

TEST("Línea perdida por la derecha gira a la derecha") {
  Rig rig;
  rig.bus.lineAt(13, 14);
  rig.follower.start(150, 0);
  rig.follower.update(1000);
  rig.bus.fill(30);
  rig.follower.update(1005);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, rig.config.lostLine.outerMotorPwm);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, rig.config.lostLine.innerMotorPwm);
}

TEST("requestStop frena en rampa y termina la carrera a los 500 ms") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(200, 0);
  rig.follower.update(1000);
  rig.follower.requestStop(1000);
  EXPECT_TRUE(rig.follower.isStopping());
  rig.follower.update(1250);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 100);
  rig.follower.update(1500);
  EXPECT_FALSE(rig.follower.isRunning());
}

TEST("El impeller sube en rampa durante la carrera") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  rig.follower.update(125);
  EXPECT_EQ(rig.drivetrain.impellerMotor.pwm, rig.config.impeller.pwm / 2);
  rig.follower.update(400);
  EXPECT_EQ(rig.drivetrain.impellerMotor.pwm, rig.config.impeller.pwm);
}
