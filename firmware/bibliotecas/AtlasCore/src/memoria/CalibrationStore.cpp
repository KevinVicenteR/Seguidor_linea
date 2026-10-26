#include "CalibrationStore.h"

namespace atlas {

constexpr uint8_t CalibrationStore::kSignature;
constexpr uint8_t CalibrationStore::kVersion;
constexpr uint16_t CalibrationStore::kBaseAddress;

namespace {
constexpr uint16_t kHeaderSize = 2;
constexpr uint16_t kChecksumOffset = kHeaderSize + 2 * kSensorCount;
}

void CalibrationStore::writeByte(uint16_t offset, uint8_t value, uint8_t* checksum) {
  if (storage_.read(kBaseAddress + offset) != value) storage_.write(kBaseAddress + offset, value);
  *checksum ^= value;
}

void CalibrationStore::save(const LineSensorArray& sensors) {
  uint8_t checksum = 0;
  writeByte(0, kSignature, &checksum);
  writeByte(1, kVersion, &checksum);
  for (uint8_t i = 0; i < kSensorCount; i++) writeByte(kHeaderSize + i, sensors.minValues()[i], &checksum);
  for (uint8_t i = 0; i < kSensorCount; i++) writeByte(kHeaderSize + kSensorCount + i, sensors.maxValues()[i], &checksum);
  uint8_t ignored = 0;
  writeByte(kChecksumOffset, checksum, &ignored);
}

bool CalibrationStore::load(LineSensorArray& sensors) {
  uint8_t data[kChecksumOffset + 1];
  uint8_t checksum = 0;
  for (uint16_t i = 0; i <= kChecksumOffset; i++) {
    data[i] = storage_.read(kBaseAddress + i);
    if (i < kChecksumOffset) checksum ^= data[i];
  }
  if (data[0] != kSignature || data[1] != kVersion || data[kChecksumOffset] != checksum) return false;

  sensors.restoreCalibration(data + kHeaderSize, data + kHeaderSize + kSensorCount);
  return true;
}

}
