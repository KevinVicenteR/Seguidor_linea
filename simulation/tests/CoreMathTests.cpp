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
  EXPECT_EQ(pd.update(1000), 50 + 350);  // primer ciclo: delta = error
  EXPECT_EQ(pd.update(1000), 50);        // error constante: sin derivativo
  EXPECT_EQ(pd.update(800), 40 - 70);
}

TEST("PdController satura la salida") {
  PdController pd(1.0f, 0.0f, 300);
  EXPECT_EQ(pd.update(7500), 300);
  EXPECT_EQ(pd.update(-7500), -300);
}

TEST("PdController::reset borra la memoria del error") {
  PdController pd(0.0f, 1.0f, 1000);
  pd.update(500);
  pd.reset();
  EXPECT_EQ(pd.update(500), 500);
}
