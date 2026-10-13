#include "SimHardware.h"

#include <algorithm>
#include <cmath>

namespace sim {

constexpr uint32_t SimClock::kMaxChunkUs;

void SimClock::advanceUs(uint64_t us) {
  while (us > 0) {
    const uint32_t chunk = static_cast<uint32_t>(std::min<uint64_t>(us, kMaxChunkUs));
    nowUs_ += chunk;
    us -= chunk;
    if (listener_ != nullptr) listener_->onTimeAdvanced(nowUs_, chunk);
  }
}

SimLineSensorBus::SimLineSensorBus(const Track& track, const RobotModel& robot, SimClock& clock,
                                   const SensorModel& model, atlas::LineColor trackLineColor, uint32_t seed)
    : track_(track),
      robot_(robot),
      clock_(clock),
      model_(model),
      trackLineColor_(trackLineColor),
      random_(seed),
      noise_(0.0, model.noiseSigma) {
  std::uniform_real_distribution<double> gain(1.0 - model.gainSpread, 1.0 + model.gainSpread);
  std::uniform_real_distribution<double> offset(-8.0, 8.0);
  for (uint8_t i = 0; i < atlas::kSensorCount; i++) {
    gain_[i] = gain(random_);
    offset_[i] = offset(random_);
  }
}

// Fracción del área del sensor que cae sobre la línea (0 = fondo, 1 = línea).
double SimLineSensorBus::reflectance(const Vec2& point) const {
  const double distance = track_.distanceToLine(point);
  const double edge = track_.lineWidth() / 2.0;
  const double coverage = (edge + model_.spotRadius - distance) / (2.0 * model_.spotRadius);
  return std::max(0.0, std::min(1.0, coverage));
}

uint8_t SimLineSensorBus::read(uint8_t channel) {
  clock_.advanceUs(model_.adcReadUs);
  if (channel == model_.deadSensor) return static_cast<uint8_t>(model_.deadSensorValue);

  double onLine = reflectance(robot_.sensorPosition(channel, atlas::kSensorCount));
  if (trackLineColor_ == atlas::LineColor::Black) onLine = 1.0 - onLine;  // fondo blanco, línea negra

  const double span = (model_.lineLevel - model_.surfaceLevel) * model_.contrast;
  const double adc = (model_.surfaceLevel + span * onLine) * gain_[channel] + offset_[channel] +
                     model_.ambientOffset + noise_(random_);
  return static_cast<uint8_t>(std::max(0.0, std::min(255.0, std::round(adc))));
}

}  // namespace sim
