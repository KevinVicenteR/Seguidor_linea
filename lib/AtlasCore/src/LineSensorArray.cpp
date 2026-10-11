#include "LineSensorArray.h"

namespace atlas {

constexpr uint8_t LineSensorArray::kAdcMax;

LineSensorArray::LineSensorArray(ILineSensorBus& bus, const SensorConfig& config)
    : bus_(bus), config_(config) {
  resetCalibration();
}

void LineSensorArray::setLineColor(LineColor color) { lineColor_ = color; }

void LineSensorArray::readRaw() {
  const bool invert = lineColor_ == LineColor::Black;
  for (uint8_t i = 0; i < kSensorCount; i++) {
    const uint8_t adc = bus_.read(i);
    values_[i] = invert ? kAdcMax - adc : adc;
  }
}

void LineSensorArray::readAveraged() {
  uint16_t sum[kSensorCount] = {};
  for (uint8_t sample = 0; sample < config_.averageSamples; sample++) {
    readRaw();
    for (uint8_t i = 0; i < kSensorCount; i++) sum[i] += values_[i];
  }
  for (uint8_t i = 0; i < kSensorCount; i++) values_[i] = sum[i] / config_.averageSamples;
}

void LineSensorArray::resetCalibration() {
  for (uint8_t i = 0; i < kSensorCount; i++) {
    max_[i] = 0;
    min_[i] = kAdcMax;
    threshold_[i] = kAdcMax;
  }
}

// Solo mueve el máximo (o mínimo) si TODAS las muestras del colector lo
// superan, lo que filtra picos de ruido durante la pasada de calibración.
void LineSensorArray::collectCalibrationSamples() {
  uint8_t highest[kSensorCount];
  uint8_t lowest[kSensorCount];

  for (uint8_t sample = 0; sample < config_.collectorSamples; sample++) {
    readAveraged();
    for (uint8_t i = 0; i < kSensorCount; i++) {
      if (sample == 0 || values_[i] > highest[i]) highest[i] = values_[i];
      if (sample == 0 || values_[i] < lowest[i]) lowest[i] = values_[i];
    }
  }

  for (uint8_t i = 0; i < kSensorCount; i++) {
    if (lowest[i] > max_[i]) max_[i] = lowest[i];
    if (highest[i] < min_[i]) min_[i] = highest[i];
  }
}

void LineSensorArray::finishCalibration() {
  for (uint8_t i = 0; i < kSensorCount; i++) {
    if (max_[i] <= min_[i]) {
      threshold_[i] = kAdcMax;  // sensor sin calibrar: nunca reporta línea
      continue;
    }
    const uint16_t span = max_[i] - min_[i];
    threshold_[i] = min_[i] + span * config_.thresholdPercent / 100;
  }
}

uint8_t LineSensorArray::scaleToCalibratedRange(uint8_t sensor, uint8_t raw) const {
  if (raw <= threshold_[sensor]) return 0;
  if (raw >= max_[sensor]) return kAdcMax;
  const uint16_t range = max_[sensor] - threshold_[sensor];
  return static_cast<uint8_t>(static_cast<uint32_t>(raw - threshold_[sensor]) * kAdcMax / range);
}

void LineSensorArray::readCalibrated() {
  readRaw();
  for (uint8_t i = 0; i < kSensorCount; i++) values_[i] = scaleToCalibratedRange(i, values_[i]);
}

LineReading LineSensorArray::readLine() {
  readCalibrated();

  if (!config_.trackNearestCluster) return weightedReading({0, kSensorCount - 1});

  const Cluster cluster = nearestCluster();
  LineReading reading = weightedReading(cluster);
  if (!reading.onLine) {
    wasOnLine_ = false;
    heldReads_ = 0;
    return reading;
  }

  // En un cruce se mantiene la última posición: el robot sigue recto.
  if (isAmbiguous(cluster, reading.position) && heldReads_ < config_.maxHeldReads) {
    reading.position = lastPosition_;
    heldReads_++;
  } else {
    lastPosition_ = reading.position;
    heldReads_ = 0;
  }
  wasOnLine_ = true;
  return reading;
}

void LineSensorArray::resetTracking() {
  lastPosition_ = kLinePositionCenter;
  wasOnLine_ = false;
  heldReads_ = 0;
}

// Grupo demasiado ancho para ser una sola línea, o salto imposible de posición.
bool LineSensorArray::isAmbiguous(Cluster cluster, uint16_t position) const {
  const bool tooWide = cluster.last - cluster.first + 1 > config_.maxLineWidthSensors;
  const int32_t jump = static_cast<int32_t>(position) - static_cast<int32_t>(lastPosition_);
  const bool jumped = wasOnLine_ && (jump > config_.maxPositionJump || -jump > config_.maxPositionJump);
  return tooWide || jumped;
}

// Promedio ponderado de los sensores [first, last] (posición 0 ... 15000).
LineReading LineSensorArray::weightedReading(Cluster cluster) const {
  uint32_t weighted = 0;
  uint32_t sum = 0;
  LineReading reading;

  for (uint8_t i = cluster.first; i <= cluster.last; i++) {
    if (values_[i] == 0) continue;
    reading.onLine = true;
    weighted += static_cast<uint32_t>(values_[i]) * i * kSensorPositionScale;
    sum += values_[i];
  }

  reading.position = weighted ? static_cast<uint16_t>(weighted / sum) : 0;
  reading.leftEdgeDetected = cluster.first <= 1 && (values_[0] || values_[1]);
  reading.rightEdgeDetected =
      cluster.last >= kSensorCount - 2 && (values_[kSensorCount - 2] || values_[kSensorCount - 1]);
  return reading;
}

// Grupo de sensores contiguos activos cuyo centro está más cerca de la
// última posición conocida. Si no hay ninguno devuelve toda la barra.
LineSensorArray::Cluster LineSensorArray::nearestCluster() const {
  Cluster best = {0, kSensorCount - 1};
  uint32_t bestDistance = 0xFFFFFFFFUL;

  uint8_t i = 0;
  while (i < kSensorCount) {
    if (values_[i] == 0) {
      i++;
      continue;
    }
    Cluster cluster = {i, i};
    while (cluster.last + 1 < kSensorCount && values_[cluster.last + 1] != 0) cluster.last++;

    const int32_t center = static_cast<int32_t>(cluster.first + cluster.last) * kSensorPositionScale / 2;
    const int32_t delta = center - static_cast<int32_t>(lastPosition_);
    const uint32_t distance = static_cast<uint32_t>(delta < 0 ? -delta : delta);
    if (distance < bestDistance) {
      bestDistance = distance;
      best = cluster;
    }
    i = cluster.last + 1;
  }
  return best;
}

}  // namespace atlas
