#include "../AtlasApp.h"

namespace atlas {
void AtlasApp::updateImpellerPreSpin() {
  const uint32_t now = clock_.millis();
  const RobotConfig& nextConfig = rounds_.config(hasTrackMap(), config_, fastConfig_, thirdConfig_);
  drivetrain_.impeller().setPwm(turbine_.prepare(input_.startReady(), now, nextConfig.impeller));
  leds_.set(Led::Led2, turbine_.atSpeed(now));
}
}  // namespace atlas
