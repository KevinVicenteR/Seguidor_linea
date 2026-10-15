/*
  LineSensorArray.h - Barra de 16 sensores QRE1113: lectura, calibración
  y cálculo de la posición de la línea (0 ... 15000, centro = 7500).
*/
#pragma once

#include <stdint.h>

#include "Config.h"
#include "Hal.h"

namespace atlas {

struct LineReading {
  bool onLine = false;
  uint16_t position = 0;
  bool leftEdgeDetected = false;   // sensores 0-1 ven la línea
  bool rightEdgeDetected = false;  // sensores 14-15 ven la línea
  bool ambiguous = false;          // cruce/esquina: se mantuvo la posición anterior
  bool otherLineLeft = false;      // hay otra línea a la izquierda del grupo seguido
  bool otherLineRight = false;     // ... o a la derecha (ramas de picos y cruces)

  // Posición válida para el PD: la línea está dentro de la barra y no
  // saturada en un extremo (en el extremo ya no se puede estimar el error).
  bool isTrackable() const { return onLine && position != 0 && position != kLinePositionMax; }
};

class LineSensorArray {
 public:
  LineSensorArray(ILineSensorBus& bus, const SensorConfig& config);

  LineColor lineColor() const { return config_.lineColor; }

  void readRaw();
  void readCalibrated();
  LineReading readLine();
  void resetTracking();
  void setHeldReadsLimit(uint16_t reads) { heldReadsLimit_ = reads; }

  void resetCalibration();
  void collectCalibrationSamples();
  void finishCalibration();
  // Restaura una calibración guardada (mínimos y máximos por sensor).
  void restoreCalibration(const uint8_t* minValues, const uint8_t* maxValues);
  uint8_t calibratedSensorCount() const;
  bool isSensorCalibrated(uint8_t sensor) const { return max_[sensor] >= min_[sensor] + config_.minCalibrationSpan; }
  bool isCalibrationValid() const { return calibratedSensorCount() >= config_.requiredCalibratedSensors; }

  const uint8_t* values() const { return values_; }
  // Lecturas normalizadas sin umbral (0 = fondo, 127 = línea): entrada de la
  // red neuronal de posición. Solo se calculan con neuralPosition activo.
  const int8_t* normalizedValues() const { return normalized_; }
  const uint8_t* minValues() const { return min_; }
  const uint8_t* maxValues() const { return max_; }
  const uint8_t* thresholds() const { return threshold_; }

 private:
  static constexpr uint8_t kAdcMax = 0xFF;

  struct Cluster {
    uint8_t first;
    uint8_t last;
  };

  void readAveraged();
  uint8_t scaleToCalibratedRange(uint8_t sensor, uint8_t raw) const;
  void fillDamagedSensors();
  LineReading weightedReading(Cluster cluster) const;
  bool canUseNeuralPosition(Cluster cluster) const;
  uint16_t neuralPosition(Cluster cluster, uint16_t weighted) const;
  void normalizeRaw();
  uint16_t clusterIntensity(Cluster cluster) const;
  Cluster nearestCluster() const;
  bool isAmbiguous(Cluster cluster, uint16_t position) const;
  bool isReacquiring() const { return readsSinceReacquired_ <= config_.reacquireFreeReads; }
  void rememberNearbyLines(const LineReading& reading);
  bool otherLineNearby() const;
  void learnNormalShape(uint8_t width, uint16_t intensity);

  ILineSensorBus& bus_;
  const SensorConfig& config_;

  uint8_t values_[kSensorCount] = {};
  uint8_t min_[kSensorCount] = {};
  uint8_t max_[kSensorCount] = {};
  uint8_t threshold_[kSensorCount] = {};
  uint8_t normalizeScale_[kSensorCount] = {};  // (127 << 6) / (máx - mín)
  int8_t normalized_[kSensorCount] = {};
  uint16_t lastPosition_ = kLinePositionCenter;
  bool wasOnLine_ = false;
  uint8_t readsSinceReacquired_ = 0;
  uint16_t heldReads_ = 0;
  static constexpr uint8_t kSettledReads = 20;  // ~15 ms de línea única y limpia
  uint8_t cleanHeldReads_ = 0;
  uint16_t heldReadsLimit_;
  uint16_t readsSinceOtherLine_ = 0xFFFF;
  static constexpr uint8_t kWidthWindow = 32;
  uint8_t normalWidth_ = 3;  // ancho (sensores) de la línea sola, medido sin otras líneas cerca
  uint8_t windowWidth_ = 0;
  uint8_t widthSamples_ = 0;
  uint16_t normalIntensity_ = 3 * 255;  // suma de valores de una línea sola
  uint16_t windowIntensity_ = 0;
};

}  // namespace atlas
