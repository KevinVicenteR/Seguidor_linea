#include "LineSensorArray.h"

#include "NeuralPositionWeights.h"
#include "TinyMlp.h"

namespace atlas {

constexpr uint8_t LineSensorArray::kAdcMax;
constexpr uint8_t LineSensorArray::kWidthWindow;
constexpr uint8_t LineSensorArray::kSettledReads;

LineSensorArray::LineSensorArray(ILineSensorBus& bus, const SensorConfig& config)
    : bus_(bus), config_(config), heldReadsLimit_(config.maxHeldReads) {
  resetCalibration();
}

void LineSensorArray::readRaw() {
  const bool invert = config_.lineColor == LineColor::Black;
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
      threshold_[i] = kAdcMax;
      normalizeScale_[i] = 0;
      continue;
    }
    const uint16_t span = max_[i] - min_[i];
    threshold_[i] = min_[i] + span * config_.thresholdPercent / 100;
    normalizeScale_[i] = static_cast<uint8_t>(span >= 32 ? (127U << 6) / span : 0);
  }
}

void LineSensorArray::restoreCalibration(const uint8_t* minValues, const uint8_t* maxValues) {
  for (uint8_t i = 0; i < kSensorCount; i++) {
    min_[i] = minValues[i];
    max_[i] = maxValues[i];
  }
  finishCalibration();
}

uint8_t LineSensorArray::calibratedSensorCount() const {
  uint8_t count = 0;
  for (uint8_t i = 0; i < kSensorCount; i++) {
    if (isSensorCalibrated(i)) count++;
  }
  return count;
}

uint8_t LineSensorArray::scaleToCalibratedRange(uint8_t sensor, uint8_t raw) const {
  if (raw <= threshold_[sensor]) return 0;
  if (raw >= max_[sensor]) return kAdcMax;
  const uint16_t range = max_[sensor] - threshold_[sensor];
  return static_cast<uint8_t>(static_cast<uint32_t>(raw - threshold_[sensor]) * kAdcMax / range);
}

void LineSensorArray::readCalibrated() {
  readRaw();
  if (config_.neuralPosition || config_.exposeNormalizedValues) normalizeRaw();
  for (uint8_t i = 0; i < kSensorCount; i++) values_[i] = scaleToCalibratedRange(i, values_[i]);
  fillDamagedSensors();
}

void LineSensorArray::normalizeRaw() {
  for (uint8_t i = 0; i < kSensorCount; i++) {
    const uint8_t aboveMin = values_[i] > min_[i] ? values_[i] - min_[i] : 0;
    const uint16_t scaled = static_cast<uint16_t>(aboveMin * normalizeScale_[i]) >> 6;
    normalized_[i] = static_cast<int8_t>(scaled > 127 ? 127 : scaled);
  }
}

void LineSensorArray::fillDamagedSensors() {
  for (uint8_t i = 1; i + 1 < kSensorCount; i++) {
    if (isSensorCalibrated(i) || !isSensorCalibrated(i - 1) || !isSensorCalibrated(i + 1)) continue;
    values_[i] = static_cast<uint8_t>((static_cast<uint16_t>(values_[i - 1]) + values_[i + 1]) / 2);
    normalized_[i] = static_cast<int8_t>((normalized_[i - 1] + normalized_[i + 1]) / 2);
  }
}

LineReading LineSensorArray::readLine() {
  readCalibrated();

  if (!config_.trackNearestCluster) return weightedReading({0, kSensorCount - 1});

  const Cluster cluster = nearestCluster();
  LineReading reading = weightedReading(cluster);
  for (uint8_t i = 0; i < kSensorCount; i++) {
    if (values_[i] == 0) continue;
    if (i < cluster.first) reading.otherLineLeft = true;
    if (i > cluster.last) reading.otherLineRight = true;
  }
  if (!reading.onLine) {
    wasOnLine_ = false;
    heldReads_ = 0;
    readsSinceReacquired_ = 0;
    return reading;
  }
  if (readsSinceReacquired_ < 0xFF) readsSinceReacquired_++;

  rememberNearbyLines(reading);
  const uint8_t width = cluster.last - cluster.first + 1;
  const uint16_t intensity = clusterIntensity(cluster);
  const bool merging =
      otherLineNearby() && (width > normalWidth_ || intensity > normalIntensity_ + normalIntensity_ / 5);

  const bool ambiguous = !isReacquiring() && (merging || isAmbiguous(cluster, reading.position));
  const bool singleCleanLine = !reading.otherLineLeft && !reading.otherLineRight && width <= normalWidth_ &&
                               intensity <= normalIntensity_ + normalIntensity_ / 5;
  cleanHeldReads_ = ambiguous && singleCleanLine && cleanHeldReads_ < 0xFF ? cleanHeldReads_ + 1 : 0;
  const bool settled = cleanHeldReads_ >= kSettledReads;
  if (ambiguous && !settled && heldReads_ < heldReadsLimit_) {
    reading.position = lastPosition_;
    reading.ambiguous = true;
    heldReads_++;
  } else {
    lastPosition_ = reading.position;
    heldReads_ = 0;
    if (!otherLineNearby()) learnNormalShape(width, intensity);
  }
  wasOnLine_ = true;
  return reading;
}

void LineSensorArray::rememberNearbyLines(const LineReading& reading) {
  if (reading.otherLineLeft || reading.otherLineRight) {
    readsSinceOtherLine_ = 0;
  } else if (readsSinceOtherLine_ < 0xFFFF) {
    readsSinceOtherLine_++;
  }
}

bool LineSensorArray::otherLineNearby() const { return readsSinceOtherLine_ <= heldReadsLimit_ / 4; }

uint16_t LineSensorArray::clusterIntensity(Cluster cluster) const {
  uint16_t sum = 0;
  for (uint8_t i = cluster.first; i <= cluster.last; i++) sum += values_[i];
  return sum;
}

void LineSensorArray::learnNormalShape(uint8_t width, uint16_t intensity) {
  if (width > windowWidth_) windowWidth_ = width;
  if (intensity > windowIntensity_) windowIntensity_ = intensity;
  if (width > normalWidth_) normalWidth_ = width;
  if (intensity > normalIntensity_) normalIntensity_ = intensity;
  if (++widthSamples_ >= kWidthWindow) {
    normalWidth_ = windowWidth_;
    normalIntensity_ = windowIntensity_;
    windowWidth_ = 0;
    windowIntensity_ = 0;
    widthSamples_ = 0;
  }
}

void LineSensorArray::resetTracking() {
  lastPosition_ = kLinePositionCenter;
  wasOnLine_ = false;
  readsSinceReacquired_ = 0xFF;
  heldReads_ = 0;
  readsSinceOtherLine_ = 0xFFFF;
  normalWidth_ = config_.maxLineWidthSensors;
  windowWidth_ = 0;
  widthSamples_ = 0;
  normalIntensity_ = 3 * kAdcMax;
  windowIntensity_ = 0;
}

bool LineSensorArray::isAmbiguous(Cluster cluster, uint16_t position) const {
  const uint8_t widest = config_.maxLineWidthSensors + (otherLineNearby() ? 0 : 1);
  const bool tooWide = cluster.last - cluster.first + 1 > widest;
  const int32_t jump = static_cast<int32_t>(position) - static_cast<int32_t>(lastPosition_);
  const bool jumped = wasOnLine_ && (jump > config_.maxPositionJump || -jump > config_.maxPositionJump);
  return tooWide || jumped;
}

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
  if (reading.onLine && canUseNeuralPosition(cluster)) reading.position = neuralPosition(cluster, reading.position);
  reading.leftEdgeDetected = cluster.first <= 1 && (values_[0] || values_[1]);
  reading.rightEdgeDetected =
      cluster.last >= kSensorCount - 2 && (values_[kSensorCount - 2] || values_[kSensorCount - 1]);
  return reading;
}

bool LineSensorArray::canUseNeuralPosition(Cluster cluster) const {
  const uint8_t width = cluster.last - cluster.first + 1;
  return config_.neuralPosition && cluster.first > 0 && cluster.last < kSensorCount - 1 &&
         width <= config_.maxLineWidthSensors;
}

uint16_t LineSensorArray::neuralPosition(Cluster cluster, uint16_t weighted) const {
  const int8_t center = static_cast<int8_t>((cluster.first + cluster.last) / 2);
  int8_t input[neural::kPositionInputs];
  for (int8_t k = 0; k < static_cast<int8_t>(neural::kPositionInputs); k++) {
    const int8_t sensor = center + k - static_cast<int8_t>(neural::kPositionInputs / 2);
    const bool visible = sensor >= static_cast<int8_t>(cluster.first) - 1 && sensor <= cluster.last + 1 &&
                         sensor >= 0 && sensor < static_cast<int8_t>(kSensorCount);
    input[k] = visible ? normalized_[sensor] : 0;
  }
  int32_t offset = 0;
  TinyMlp(neural::kPositionNet).evaluate(input, &offset);

  const int32_t correction = config_.neuralMaxCorrection;
  int32_t lowest = cluster.first * static_cast<int32_t>(kSensorPositionScale) - kSensorPositionScale / 2;
  int32_t highest = cluster.last * static_cast<int32_t>(kSensorPositionScale) + kSensorPositionScale / 2;
  if (lowest < weighted - correction) lowest = weighted - correction;
  if (highest > weighted + correction) highest = weighted + correction;
  int32_t position = center * static_cast<int32_t>(kSensorPositionScale) + offset;
  if (position < lowest) position = lowest;
  if (position > highest) position = highest;
  return static_cast<uint16_t>(position);
}

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

}
