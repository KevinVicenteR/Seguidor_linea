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

  void resetCalibration();
  void collectCalibrationSamples();
  void finishCalibration();
  uint8_t calibratedSensorCount() const;
  bool isCalibrationValid() const { return calibratedSensorCount() >= config_.requiredCalibratedSensors; }

  const uint8_t* values() const { return values_; }
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
  LineReading weightedReading(Cluster cluster) const;
  Cluster nearestCluster() const;
  bool isAmbiguous(Cluster cluster, uint16_t position) const;

  ILineSensorBus& bus_;
  const SensorConfig& config_;

  uint8_t values_[kSensorCount] = {};
  uint8_t min_[kSensorCount] = {};
  uint8_t max_[kSensorCount] = {};
  uint8_t threshold_[kSensorCount] = {};
  uint16_t lastPosition_ = kLinePositionCenter;
  bool wasOnLine_ = false;
  uint8_t heldReads_ = 0;
};

}  // namespace atlas
