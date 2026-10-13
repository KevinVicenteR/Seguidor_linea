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
  // Límite de lecturas consecutivas manteniendo la posición (~110 ms).
  uint8_t maxHeldReads = 150;
};

struct ControlConfig {
  float kp = 0.05f;
  float kd = 0.35f;
  int16_t maxCorrectionPwm = 1000;
  // Frenado en curva: con la línea en el extremo de la barra la velocidad
  // crucero se reduce este porcentaje (proporcional al error). 0 = original.
  uint8_t curveSlowdownPercent = 40;
  int16_t maxForwardPwm = 255;
  int16_t maxBackwardPwm = -120;
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
  int16_t innerMotorPwm = -120;
  uint16_t innerBrakeMs = 35;
  // Huecos en la línea: si se pierde la línea después de ir recto y centrado
  // (en una recta), el robot sigue recto hasta gapBridgeMs antes de buscarla
  // girando. En curvas, rizos y picos la línea nunca está centrada tanto tiempo.
  uint16_t gapBridgeMs = 200;
  int16_t gapCenterTolerance = 1500;  // |error| máximo para considerarse "centrado"
  uint16_t gapStraightMs = 60;        // tiempo centrado exigido antes del hueco
};

struct RobotConfig {
  SensorConfig sensors;
  ControlConfig control;
  SpeedConfig speed;
  ImpellerConfig impeller;
  LostLineConfig lostLine;
  bool motorsEnabled = true;
};

}  // namespace atlas
