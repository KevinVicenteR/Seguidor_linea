/*
  Config.h - Parámetros del robot ATLAS agrupados por responsabilidad.

  Los valores por defecto replican el firmware original ATLAS 1.4.3.
  Cada módulo recibe solo la sección de configuración que necesita
  (inyección de dependencias), lo que permite que el simulador y las
  pruebas usen configuraciones distintas sin tocar el código de control.
*/
#pragma once

#include <stdint.h>

namespace atlas {

constexpr uint8_t kSensorCount = 16;
constexpr uint16_t kSensorPositionScale = 1000;
constexpr uint16_t kLinePositionMax = (kSensorCount - 1) * kSensorPositionScale;  // 15000
constexpr uint16_t kLinePositionCenter = kLinePositionMax / 2;                    // 7500
constexpr int16_t kPwmResolution = 255;

enum class LineColor : uint8_t { White, Black };

struct SensorConfig {
  // Único modo de operación: línea negra sobre fondo blanco.
  LineColor lineColor = LineColor::Black;
  uint8_t thresholdPercent = 50;
  uint8_t averageSamples = 8;     // lecturas promediadas por muestra de calibración
  uint8_t collectorSamples = 10;  // muestras consecutivas exigidas para mover min/max
  // Un sensor queda calibrado si vio línea y fondo con al menos esta diferencia.
  uint8_t minCalibrationSpan = 40;
  // Sensores calibrados necesarios para aceptar la calibración.
  uint8_t requiredCalibratedSensors = kSensorCount;
  // Con varias líneas bajo la barra (cruces, rizos) usar solo el grupo de
  // sensores más cercano a la posición anterior. false = algoritmo original.
  bool trackNearestCluster = true;
  // Un grupo más ancho que esto no es una línea sino un cruce o esquina: se
  // mantiene la última posición (el robot sigue recto) hasta que se separen.
  uint8_t maxLineWidthSensors = 3;
  // La línea no puede desplazarse más que esto entre dos lecturas (750 us);
  // un salto mayor indica que otra línea se unió al grupo.
  uint16_t maxPositionJump = 1000;
  // Límite de lecturas consecutivas manteniendo la posición, definido a PWM
  // de referencia; el seguidor lo escala con la velocidad para que equivalga
  // siempre a la misma distancia (~30 cm), también a velocidad baja.
  uint16_t maxHeldReads = 150;
  int16_t heldReadsReferencePwm = 250;
};

struct ControlConfig {
  float kp = 0.09f;
  float kd = 1.15f;
  int16_t maxCorrectionPwm = 1000;
  // Filtro del derivativo (media exponencial): 0 = sin filtro, n = peso 1/2^n.
  uint8_t derivativeFilterShift = 2;
  // Frenado en curva: con la línea en el extremo de la barra la velocidad
  // crucero se reduce este porcentaje (proporcional al error). 0 = original.
  uint8_t curveSlowdownPercent = 8;
  int16_t maxForwardPwm = 255;
  int16_t maxBackwardPwm = -210;
  uint16_t loopPeriodUs = 750;
};

struct SpeedConfig {
  int16_t basePwm = 100;
  int16_t minPwm = 60;
  int16_t maxPwm = 250;
  int16_t stepPwm = 10;
  uint16_t runRampMs = 150;    // subida de velocidad al arrancar
  uint16_t stopBrakeMs = 500;  // rampa de frenado al quitar GO
};

struct ImpellerConfig {
  int16_t pwm = 200;
  uint16_t startRampMs = 1000;  // rampa con la señal READY
  uint16_t runRampMs = 250;     // rampa al iniciar la carrera
};

struct LostLineConfig {
  int16_t outerMotorPwm = 255;
  int16_t innerMotorPwm = -210;
  uint16_t innerBrakeMs = 45;
  // Huecos en la línea: si se pierde la línea después de ir recto y centrado
  // (en una recta), el robot sigue recto hasta gapBridgeMs antes de buscarla
  // girando. En curvas, rizos y picos la línea nunca está centrada tanto tiempo.
  uint16_t gapBridgeMs = 200;
  int16_t gapCenterTolerance = 1500;  // |error| máximo para considerarse "centrado"
  uint16_t gapStraightMs = 60;        // tiempo centrado exigido antes del hueco
};

// Frenado anticipado en esquinas: la barra (18 cm delante del eje) detecta la
// esquina antes que las ruedas; se reduce la velocidad mientras dura y luego
// se recupera en rampa. Afecta también a la velocidad de búsqueda de línea.
// Desactivado: con el optimizador (simulation/tools/atlas_tune) no mejoró la
// precisión frente al frenado en curva proporcional al error.
struct CornerConfig {
  bool enabled = false;
  int16_t triggerError = 3500;   // |error| que se considera esquina/curva cerrada
  uint8_t speedPercent = 55;     // velocidad durante la esquina (% del crucero)
  uint16_t holdMs = 40;          // se mantiene tras la última detección
  uint16_t recoveryMs = 120;     // rampa de regreso a la velocidad crucero
};

struct RobotConfig {
  SensorConfig sensors;
  ControlConfig control;
  SpeedConfig speed;
  ImpellerConfig impeller;
  LostLineConfig lostLine;
  CornerConfig corner;
  bool motorsEnabled = true;
};

}  // namespace atlas
