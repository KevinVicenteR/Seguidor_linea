#include "PdController.h"
#include "Ramp.h"
#include "TestFramework.h"

using namespace atlas;

TEST("LinearRamp interpola y se satura al final") {
  LinearRamp ramp;
  ramp.start(1000, 200, 0, 100);
  EXPECT_EQ(ramp.valueAt(1000), 0);
  EXPECT_EQ(ramp.valueAt(1100), 50);
  EXPECT_EQ(ramp.valueAt(1200), 100);
  EXPECT_EQ(ramp.valueAt(5000), 100);
  EXPECT_TRUE(ramp.isFinished(1200));
  EXPECT_FALSE(ramp.isFinished(1199));
}

TEST("LinearRamp descendente para el frenado") {
  LinearRamp ramp;
  ramp.start(0, 500, 200, 0);
  EXPECT_EQ(ramp.valueAt(250), 100);
  EXPECT_EQ(ramp.valueAt(500), 0);
}

TEST("LinearRamp con duración cero devuelve el destino") {
  LinearRamp ramp;
  ramp.start(10, 0, 0, 77);
  EXPECT_EQ(ramp.valueAt(10), 77);
}

TEST("PdController: término proporcional y derivativo") {
  PdController pd(0.05f, 0.35f, 1000);
  EXPECT_EQ(pd.update(0), 0);
  EXPECT_EQ(pd.update(1000), 50 + 350);
  EXPECT_EQ(pd.update(1000), 50);
  EXPECT_EQ(pd.update(800), 40 - 70);
}

TEST("PdController satura la salida") {
  PdController pd(1.0f, 0.0f, 300);
  EXPECT_EQ(pd.update(7500), 300);
  EXPECT_EQ(pd.update(-7500), -300);
}

TEST("PdController::reset borra la memoria del error sin tirón del derivativo") {
  PdController pd(0.0f, 1.0f, 1000);
  pd.update(500);
  pd.reset();
  EXPECT_EQ(pd.update(500), 0);
  EXPECT_EQ(pd.update(700), 200);
}

TEST("Al arrancar descentrado el PD no da un volantazo") {
  PdController pd(0.0075f, 0.86f, 1000, 1);
  EXPECT_EQ(pd.update(2000), 15);
}

TEST("PdController: el filtro suaviza un pico de ruido en el derivativo") {
  PdController raw(0.0f, 1.0f, 1000, 0);
  PdController filtered(0.0f, 1.0f, 1000, 2);
  raw.update(0);
  filtered.update(0);
  EXPECT_EQ(raw.update(400), 400);
  EXPECT_EQ(filtered.update(400), 100);
}
