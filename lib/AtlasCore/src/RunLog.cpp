#include "RunLog.h"

namespace atlas {

constexpr uint8_t RunLog::kCapacity;
constexpr uint16_t RunLog::kBaseAddress;
constexpr uint8_t RunLog::kSignature;
constexpr uint8_t RunLog::kRecordSize;

// Formato: firma, cantidad (0..8), siguiente ranura, 8 registros de 8 bytes.
namespace {
constexpr uint16_t kCountOffset = 1;
constexpr uint16_t kNextOffset = 2;
constexpr uint16_t kRecordsOffset = 3;
}  // namespace

void RunLog::writeByte(uint16_t offset, uint8_t value) {
  if (storage_.read(kBaseAddress + offset) != value) storage_.write(kBaseAddress + offset, value);
}

bool RunLog::isInitialized() { return storage_.read(kBaseAddress) == kSignature; }

void RunLog::clear() {
  writeByte(0, kSignature);
  writeByte(kCountOffset, 0);
  writeByte(kNextOffset, 0);
}

uint8_t RunLog::count() {
  if (!isInitialized()) return 0;
  const uint8_t value = storage_.read(kBaseAddress + kCountOffset);
  return value > kCapacity ? kCapacity : value;
}

uint8_t RunLog::nextSlot() {
  const uint8_t value = storage_.read(kBaseAddress + kNextOffset);
  return value >= kCapacity ? 0 : value;
}

void RunLog::append(const RunRecord& record) {
  if (!isInitialized()) clear();
  const uint8_t slot = nextSlot();
  const uint16_t base = kRecordsOffset + slot * kRecordSize;
  writeByte(base + 0, record.mode);
  writeByte(base + 1, record.failed ? 1 : 0);
  writeByte(base + 2, record.timeCentiseconds & 0xFF);
  writeByte(base + 3, record.timeCentiseconds >> 8);
  writeByte(base + 4, record.failureBin & 0xFF);
  writeByte(base + 5, record.failureBin >> 8);
  writeByte(base + 6, record.mapLength & 0xFF);
  writeByte(base + 7, record.mapLength >> 8);
  writeByte(kNextOffset, (slot + 1) % kCapacity);
  const uint8_t stored = count();
  if (stored < kCapacity) writeByte(kCountOffset, stored + 1);
}

RunRecord RunLog::at(uint8_t index) {
  const uint8_t stored = count();
  const uint8_t oldest = stored < kCapacity ? 0 : nextSlot();
  const uint16_t base = kRecordsOffset + ((oldest + index) % kCapacity) * kRecordSize;
  RunRecord record;
  record.mode = storage_.read(kBaseAddress + base);
  record.failed = storage_.read(kBaseAddress + base + 1) == 1;
  record.timeCentiseconds = storage_.read(kBaseAddress + base + 2) | (storage_.read(kBaseAddress + base + 3) << 8);
  record.failureBin = storage_.read(kBaseAddress + base + 4) | (storage_.read(kBaseAddress + base + 5) << 8);
  record.mapLength = storage_.read(kBaseAddress + base + 6) | (storage_.read(kBaseAddress + base + 7) << 8);
  return record;
}

}  // namespace atlas
