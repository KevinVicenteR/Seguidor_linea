#include "../AtlasApp.h"

namespace atlas {

void AtlasApp::saveTrackMapIfLearned() {
  const bool wasMapping = follower_.mode() == RunMode::Mapping;
  const bool learned = follower_.finishMapping();
  if (wasMapping && !learned) {
    leds_.mapRejected();
    if (storage_ != nullptr) TrackMapStore(*storage_).erase();
    return;
  }
  const bool corrected = follower_.correctMapAfterFailedRun();
  if ((learned || corrected) && storage_ != nullptr) TrackMapStore(*storage_).save(*trackMap_);
}

// Registra la carrera (tipo, tiempo, si falló y dónde) para el análisis.
void AtlasApp::logRace() {
  if (storage_ == nullptr) return;
  RunRecord record;
  record.mode = static_cast<uint8_t>(follower_.mode());
  record.failed = follower_.runFailed();
  const uint32_t centiseconds = (raceStopMs_ - raceStartMs_) / 10;
  record.timeCentiseconds = centiseconds > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(centiseconds);
  record.failureBin = follower_.failureBin();
  record.mapLength = trackMap_ != nullptr ? trackMap_->length() : 0;
  RunLog(*storage_).append(record);
}

}  // namespace atlas
