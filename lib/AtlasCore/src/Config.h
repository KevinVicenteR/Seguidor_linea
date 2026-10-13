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
  // Sensores calibrados necesarios para aceptar la calibración. Los dañados se
  // reemplazan por el promedio de sus vecinos.
  uint8_t requiredCalibratedSensors = kSensorCount - 2;
  // Con varias líneas bajo la barra (cruces, rizos) usar solo el grupo de
  // sensores más cercano a la posición anterior. false = algoritmo original.
  bool trackNearestCluster = true;
  // Un grupo más ancho que esto no es una línea sino un cruce o esquina: se
  // mantiene la última posición (el robot sigue recto) hasta que se separen.
  uint8_t maxLineWidthSensors = 3;
  // La línea no puede desplazarse más que esto entre dos lecturas (750 us);
  // un salto mayor indica que otra línea se unió al grupo.
  uint16_t maxPositionJump = 1000;
  // Al recuperar la línea tras perderla, la barra la cruza rápido: durante
  // estas lecturas (~30 ms) no se filtran saltos ni se mantiene la posición.
  uint8_t reacquireFreeReads = 40;
  // Límite de lecturas consecutivas manteniendo la posición, definido a PWM
  // de referencia; el seguidor lo escala con la velocidad para que equivalga
  // siempre a la misma distancia (~30 cm), también a velocidad baja.
  uint16_t maxHeldReads = 150;
  int16_t heldReadsReferencePwm = 250;
};

struct ControlConfig {
  float kp = 0.02f;
  float kd = 2.23f;
  int16_t maxCorrectionPwm = 1000;
  // Filtro del derivativo (media exponencial): 0 = sin filtro, n = peso 1/2^n.
  uint8_t derivativeFilterShift = 1;
  // Frenado en curva: con la línea en el extremo de la barra la velocidad
  // crucero se reduce este porcentaje (proporcional al error). 0 = original.
  uint8_t curveSlowdownPercent = 27;
  int16_t maxForwardPwm = 255;
  int16_t maxBackwardPwm = -134;
  // Compensación de zona muerta: cualquier PWM distinto de cero arranca desde
  // este mínimo (con la succión los motores no giran con PWM bajos).
  int16_t minMotorPwm = 40;
  uint16_t loopPeriodUs = 750;
};

struct SpeedConfig {
  int16_t basePwm = 100;
  int16_t minPwm = 60;
  int16_t maxPwm = 250;
  int16_t stepPwm = 10;
  uint16_t runRampMs = 150;    // de la velocidad de arranque a la crucero
  uint16_t stopBrakeMs = 500;  // rampa de frenado al quitar GO
  // Impulso de arranque: con la turbina el robot queda pegado a la pista y
  // necesita más fuerza para empezar a moverse.
  int16_t startKickPwm = 100;
  uint16_t startKickMs = 200;
};

struct ImpellerConfig {
  int16_t pwm = 200;
  uint16_t startRampMs = 1000;  // rampa con la señal READY (si spinWithReady)
  uint16_t runRampMs = 250;     // rampa al iniciar la carrera
  // false: la turbina solo gira con GO y se apaga en cuanto se quita GO.
  bool spinWithReady = false;
};

struct LostLineConfig {
  int16_t outerMotorPwm = 175;
  int16_t innerMotorPwm = -134;
  uint16_t innerBrakeMs = 54;
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
// Activado en el perfil Seguro: el optimizador robusto lo eligió para que un
// robot con más inercia o retraso no se pase de las esquinas.
struct CornerConfig {
  bool enabled = true;
  int16_t triggerError = 2000;   // |error| que se considera esquina/curva cerrada
  uint8_t speedPercent = 74;     // velocidad durante la esquina (% del crucero)
  uint16_t holdMs = 7;          // se mantiene tras la última detección
  uint16_t recoveryMs = 291;     // rampa de regreso a la velocidad crucero
};

// Adaptación en marcha para robots o pistas distintos de lo esperado.
struct AdaptiveConfig {
  // Auto-amortiguación (OscillationGuard)
  bool autoDamping = true;
  int16_t oscillationAmplitude = 700;  // |error| mínimo para contar un cambio de lado
  uint16_t oscillationWindowMs = 250;
  uint8_t oscillationCrossings = 4;    // cambios de lado en la ventana = zigzag
  uint8_t minGainPercent = 52;
  uint8_t gainStepDownPercent = 10;
  uint16_t gainRecoverMs = 80;  // +1 % de ganancia cada este tiempo estable

  // Gobernador de velocidad (SpeedGovernor)
  bool speedGovernor = true;
  uint8_t minSpeedPercent = 60;
  uint8_t speedDropOnLoss = 6;         // al perder la línea
  uint8_t speedDropOnOscillation = 4;  // al detectar zigzag
  uint16_t speedRecoverMs = 60;        // +1 % de velocidad cada este tiempo limpio
};

// Aprendizaje de pista: vuelta de reconocimiento y vueltas guiadas por mapa.
struct PlannerConfig {
  uint32_t binUnits = 80000;       // PWM·100 µs por tramo (≈10 cm, sin encoders)
  int16_t maxPwm = 240;            // velocidad en rectas conocidas
  int16_t offMapPwm = 110;         // velocidad si el robot sale del tramo mapeado
  int16_t fullTurnPwm = 90;        // velocidad en el giro más cerrado
  uint8_t straightCurvature = 25;  // curvatura de tramo considerada recta
  uint8_t brakePwmPerBin = 8;      // cuánto puede frenar por tramo (anticipación)
  uint8_t lookaheadBins = 25;      // tramos que mira hacia adelante (~2.5 m)
  uint16_t minimumBins = 30;       // reconocimiento mínimo para aceptar el mapa (~3 m)
  uint16_t resyncWindowBins = 25;  // corrección máxima al pasar por un hueco

  // Confianza: si en el reconocimiento el robot pasó mucho tiempo buscando la
  // línea (robot lento, con inercia o pista difícil), las vueltas guiadas
  // reducen su velocidad en proporción.
  uint16_t easyLostPerMille = 30;      // hasta 3 % sin línea = robot "fácil"
  uint8_t slowdownPerLostPerMille = 1;  // -1 % de velocidad por cada ‰ extra
  uint8_t minConfidencePercent = 70;

  // Carrera fallida: más de este tiempo seguido sin línea (en vueltas buenas
  // la racha más larga es de ~0.2 s). Un reconocimiento fallido no guarda el
  // mapa; una vuelta guiada fallida marca como peligrosa la zona donde el
  // robot se salió (frena más ahí en el siguiente intento).
  uint16_t failedRunLostMs = 1500;
  uint16_t stoppedWhileLostMs = 200;  // quitaron GO mientras buscaba la línea = se salió
  uint8_t completedMapPercent = 95;   // vuelta guiada que no recorrió esto del mapa = incompleta
  uint8_t dangerZoneBinsBefore = 10;        // ~1 m antes de donde perdió la línea
  uint8_t dangerZoneBinsAfter = 3;
  uint8_t dangerCurvatureBoost = 90;  // se suma a la curvatura de esos tramos
  uint8_t curveSpeedStepPercent = 10;  // y todas las curvas van un 10 % más lentas
  uint8_t minCurveSpeedPercent = 60;   // (las rectas no cambian)
};

struct RobotConfig {
  SensorConfig sensors;
  ControlConfig control;
  SpeedConfig speed;
  ImpellerConfig impeller;
  LostLineConfig lostLine;
  CornerConfig corner;
  AdaptiveConfig adaptive;
  PlannerConfig planner;
  bool motorsEnabled = true;
};

}  // namespace atlas
