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
    config.corner.enabled = false;
    config.speed.startKickMs = 0;
    config.speed.startKickPwm = 0;
    config.control.minMotorPwm = 0;
    sensors.resetCalibration();
    bus.fill(30);
    sensors.collectCalibrationSamples();
    bus.fill(220);
    sensors.collectCalibrationSamples();
    sensors.finishCalibration();
  }
};

void slideLineTo(Rig& rig, int from, int to, uint32_t& nowMs) {
  const int step = to > from ? 1 : -1;
  for (int first = from; first != to + step; first += step) {
    rig.bus.lineAt(first, first + 1);
    rig.follower.update(nowMs++);
  }
}

}

TEST("start habilita drivers y arranca con rampa de velocidad") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(200, 0);
  EXPECT_TRUE(rig.drivetrain.enabled);
  rig.follower.update(0);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
  rig.follower.update(75);
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
  rig.bus.lineAt(1, 2);
  rig.follower.start(150, 0);
  rig.follower.update(1000);
  rig.bus.fill(220);
  rig.follower.update(1010);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, rig.config.lostLine.innerMotorPwm);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, rig.config.lostLine.outerMotorPwm);
  rig.follower.update(1100);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, rig.config.lostLine.outerMotorPwm);
}

TEST("La recuperación prolonga la reversa tras una curva ambigua en ambos sentidos") {
  for (int first : {4, 10}) {
    Rig rig;
    rig.config.lostLine.curveBrakeMs = 115;
    rig.config.lostLine.curveWindowMs = 250;
    rig.config.lostLine.outerMotorPwm = 110;
    const int edge = first == 4 ? 1 : 13;
    rig.bus.lineAt(edge, edge + 1);
    rig.follower.start(150, 0);
    rig.follower.update(900);
    rig.sensors.resetTracking();
    rig.bus.lineAt(first, first + 1);
    for (uint32_t now = 1000; now <= 1120; now++) rig.follower.update(now);
    rig.bus.lineAt(first - 2, first + 3);
    rig.follower.update(1121);
    EXPECT_TRUE(rig.follower.lastReading().ambiguous);
    rig.bus.fill(220);
    rig.follower.update(1191);
    EXPECT_EQ(first == 4 ? rig.drivetrain.leftMotor.pwm : rig.drivetrain.rightMotor.pwm,
              rig.config.lostLine.innerMotorPwm);
    rig.follower.update(1237);
    EXPECT_EQ(first == 4 ? rig.drivetrain.leftMotor.pwm : rig.drivetrain.rightMotor.pwm, 0);
  }
}

TEST("La recuperación breve se conserva tras una pérdida repentina y al recuperar la línea") {
  Rig rig;
  rig.config.lostLine.curveBrakeMs = 115;
  rig.config.lostLine.curveWindowMs = 250;
  rig.config.lostLine.outerMotorPwm = 110;
  rig.bus.lineAt(4, 5);
  rig.follower.start(150, 0);
  rig.follower.update(1000);
  rig.bus.lineAt(2, 7);
  rig.follower.update(1001);
  EXPECT_TRUE(rig.follower.lastReading().ambiguous);
  rig.bus.fill(220);
  rig.follower.update(1071);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
  rig.bus.lineAt(4, 5);
  for (uint32_t now = 1100; now <= 1220; now++) rig.follower.update(now);
  rig.bus.lineAt(2, 7);
  rig.follower.update(1221);
  EXPECT_TRUE(rig.follower.lastReading().ambiguous);
  rig.bus.fill(220);
  rig.follower.update(1291);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, rig.config.lostLine.innerMotorPwm);
  rig.bus.lineAt(4, 5);
  for (uint32_t now = 1300; now <= 1340; now++) rig.follower.update(now);
  rig.bus.lineAt(2, 7);
  rig.follower.update(1341);
  EXPECT_TRUE(rig.follower.lastReading().ambiguous);
  rig.bus.fill(220);
  rig.follower.update(1411);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
}

TEST("Una curva ambigua con mando de giro cerrado conserva el freno breve") {
  Rig rig;
  rig.config.lostLine.curveBrakeMs = 115;
  rig.config.lostLine.curveWindowMs = 250;
  rig.config.lostLine.outerMotorPwm = 75;
  rig.bus.lineAt(1, 2);
  rig.follower.start(100, 0);
  for (uint32_t now = 1000; now <= 1120; now++) rig.follower.update(now);
  rig.bus.lineAt(0, 5);
  rig.follower.update(1121);
  EXPECT_TRUE(rig.follower.lastReading().ambiguous);
  rig.bus.fill(220);
  rig.follower.update(1191);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
}

TEST("Una curva prolongada conserva la recuperación original") {
  Rig rig;
  rig.config.lostLine.curveBrakeMs = 115;
  rig.config.lostLine.curveWindowMs = 250;
  rig.config.lostLine.outerMotorPwm = 110;
  rig.bus.lineAt(4, 5);
  rig.follower.start(150, 0);
  for (uint32_t now = 1000; now <= 1260; now++) rig.follower.update(now);
  rig.bus.lineAt(2, 7);
  rig.follower.update(1261);
  EXPECT_TRUE(rig.follower.lastReading().ambiguous);
  rig.bus.fill(220);
  rig.follower.update(1331);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
}

TEST("Una curva sin lectura ambigua conserva la recuperación original") {
  Rig rig;
  rig.config.lostLine.curveBrakeMs = 115;
  rig.config.lostLine.curveWindowMs = 250;
  rig.config.lostLine.outerMotorPwm = 110;
  rig.bus.lineAt(4, 5);
  rig.follower.start(150, 0);
  for (uint32_t now = 1000; now <= 1120; now++) rig.follower.update(now);
  EXPECT_FALSE(rig.follower.lastReading().ambiguous);
  rig.bus.fill(220);
  rig.follower.update(1190);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 0);
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

TEST("El limitador de aceleración frena la subida de velocidad, no la bajada") {
  Rig rig;
  rig.config.speed.maxAccelPwmPerS = 400;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  rig.follower.update(0);
  rig.follower.update(100);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 40);
  rig.follower.update(105);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 42);
  rig.follower.update(1000);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 150);
  rig.follower.requestStop(1000);
  rig.follower.update(1250);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 75);
}

TEST("Al dar GO el despegue sube más lento que el limitador normal") {
  Rig rig;
  rig.config.speed.maxAccelPwmPerS = 400;
  rig.config.speed.launchAccelPwmPerS = 100;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  rig.follower.update(0);
  rig.follower.update(100);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 10);
  rig.follower.update(1500);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 150);
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
  EXPECT_TRUE(curveSum < centeredSum);
}

TEST("Hueco en una recta: sigue recto y no gira") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  for (uint32_t t = 1000; t <= 1100; t++) rig.follower.update(t);
  rig.bus.fill(220);
  rig.follower.update(1150);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 150);
  EXPECT_EQ(rig.drivetrain.rightMotor.pwm, 150);
}

TEST("Punteado: tras cruzar un hueco, un trozo corto de cinta no rompe la recta") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  for (uint32_t t = 1000; t <= 1100; t++) rig.follower.update(t);
  rig.bus.fill(220);
  for (uint32_t t = 1101; t <= 1140; t++) rig.follower.update(t);
  rig.bus.lineAt(7, 8);
  for (uint32_t t = 1141; t <= 1150; t++) rig.follower.update(t);
  rig.bus.fill(220);
  rig.follower.update(1160);
  EXPECT_TRUE(rig.follower.isBridging());
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, rig.drivetrain.rightMotor.pwm);
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
  rig.bus.values[14] = rig.bus.values[15] = 30;
  rig.follower.update(1101);
  rig.bus.fill(220);
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
  rig.bus.values[14] = rig.bus.values[15] = 30;
  rig.follower.update(1001);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm + rig.drivetrain.rightMotor.pwm, 200);

  rig.bus.lineAt(7, 8);
  rig.follower.update(1001 + 40 + 50);
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

TEST("Impulso de arranque: sale con más fuerza y luego baja a la velocidad crucero") {
  Rig rig;
  rig.config.speed.startKickPwm = 130;
  rig.config.speed.startKickMs = 200;
  rig.bus.lineAt(7, 8);
  rig.follower.start(60, 1000);
  rig.follower.update(1010);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 130);
  rig.follower.update(1200 + rig.config.speed.runRampMs + 10);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 60);
}

TEST("Compensación de zona muerta: un PWM bajo sí mueve el motor y 255 sigue siendo 255") {
  Rig rig;
  rig.config.control.minMotorPwm = 45;
  rig.bus.lineAt(7, 8);
  rig.follower.start(60, 0);
  rig.follower.update(5000);
  EXPECT_EQ(rig.drivetrain.leftMotor.pwm, 45 + 60 * (255 - 45) / 255);
  rig.follower.requestStop(5000);
  rig.follower.update(5600);
  EXPECT_FALSE(rig.follower.isRunning());
}

TEST("La turbina se apaga en cuanto se pide el paro") {
  Rig rig;
  rig.bus.lineAt(7, 8);
  rig.follower.start(150, 0);
  rig.follower.update(500);
  EXPECT_EQ(rig.drivetrain.impellerMotor.pwm, rig.config.impeller.pwm);
  rig.follower.requestStop(500);
  EXPECT_EQ(rig.drivetrain.impellerMotor.pwm, 0);
  rig.follower.update(501);
  EXPECT_EQ(rig.drivetrain.impellerMotor.pwm, 0);
}
