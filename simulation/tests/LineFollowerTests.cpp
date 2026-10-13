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
    config.corner.enabled = false;  // cada prueba de esquinas lo activa explícitamente
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
  rig.bus.fill(220);  // sin línea: todo blanco
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
  rig.bus.fill(220);
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

TEST("Frenado en curva: con la línea lejos del centro baja la velocidad base") {
  Rig rig;
  rig.config.control.curveSlowdownPercent = 40;
  rig.follower.start(200, 0);
  uint32_t now = 1000;
  slideLineTo(rig, 7, 7, now);
  const int centeredSum = rig.drivetrain.leftMotor.pwm + rig.drivetrain.rightMotor.pwm;
  slideLineTo(rig, 8, 12, now);
  const int curveSum = rig.drivetrain.leftMotor.pwm + rig.drivetrain.rightMotor.pwm;
  EXPECT_EQ(centeredSum, 2 * 200);
  EXPECT_TRUE(curveSum < centeredSum);  // la suma de ruedas refleja la velocidad base
}

TEST("Hueco en una recta: sigue recto y no gira") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  for (uint32_t t = 1000; t <= 1100; t++) rig.follower.update(t);  // 100 ms recto y centrado
  rig.bus.fill(220);                                                // hueco: todo blanco
  rig.follower.update(1150);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 150);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, 150);
}

TEST("Hueco más largo que gapBridgeMs: empieza a buscar la línea") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  for (uint32_t t = 1000; t <= 1100; t++) rig.follower.update(t);
  rig.bus.fill(220);
  rig.follower.update(1100 + rig.config.lostLine.gapBridgeMs + 1);
  EXPECT_TRUE(rig.drivetrain.leftMotor.pwm != rig.drivetrain.rightMotor.pwm);
}

TEST("Línea perdida en curva (no centrada): no se trata como hueco") {
  Rig rig;
  rig.follower.start(150, 0);
  uint32_t now = 1000;
  slideLineTo(rig, 7, 12, now);
  rig.bus.fill(220);
  rig.follower.update(now + 5);
  EXPECT_TRUE(rig.drivetrain.leftMotor.pwm != rig.drivetrain.rightMotor.pwm);
}

TEST("Pico agudo: la rama que aparece a la derecha decide el giro") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  for (uint32_t t = 1000; t <= 1100; t++) rig.follower.update(t);
  rig.bus.lineAt(7, 8);
  rig.bus.values[14] = rig.bus.values[15] = 30;  // rama de regreso entrando por la derecha
  rig.follower.update(1101);
  rig.bus.fill(220);  // la línea principal termina en el pico
  rig.follower.update(1105);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, rig.config.lostLine.outerMotorPwm);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, rig.config.lostLine.innerMotorPwm);
}

TEST("Esquina detectada: baja la velocidad y la recupera en rampa") {
  Rig rig;
  rig.config.control.curveSlowdownPercent = 0;
  rig.config.corner.enabled = true;
  rig.config.corner.speedPercent = 50;
  rig.config.corner.holdMs = 40;
  rig.config.corner.recoveryMs = 100;
  rig.bus.lineAt(7, 8);
  rig.follower.start(200, 0);
  rig.follower.update(1000);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm + rig.drivetrain.rightMotor.pwm, 400);

  rig.bus.lineAt(7, 8);
  rig.bus.values[14] = rig.bus.values[15] = 30;  // aparece otra rama: esquina adelante
  rig.follower.update(1001);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm + rig.drivetrain.rightMotor.pwm, 200);  // 50 %

  rig.bus.lineAt(7, 8);
  rig.follower.update(1001 + 40 + 50);  // a mitad de la rampa de regreso
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm + rig.drivetrain.rightMotor.pwm, 300);
  rig.follower.update(1001 + 40 + 100);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm + rig.drivetrain.rightMotor.pwm, 400);
}

TEST("Frenado en esquinas desactivado: velocidad sin cambios") {
  Rig rig;
  rig.config.control.curveSlowdownPercent = 0;
  rig.config.corner.enabled = false;
  rig.bus.lineAt(7, 8);
  rig.bus.values[14] = rig.bus.values[15] = 30;
  rig.follower.start(200, 0);
  rig.follower.update(1000);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm + rig.drivetrain.rightMotor.pwm, 400);
}
