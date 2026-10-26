#include "AvrLineSensorBus.h"

namespace atlas {

namespace {
constexpr uint8_t kMuxSelectMask = 0x0F;
constexpr uint8_t kAdmuxConfigMask = 0xE0;
constexpr uint8_t kAdcChannelMask = 0x0F;
constexpr uint8_t kAdcPrescalerMask = 0x07;
constexpr uint8_t kLineSensorsAdcChannel = 4;
}

void AvrLineSensorBus::begin() {
  DDRC |= kMuxSelectMask;
  ADMUX = (ADMUX & ~kAdmuxConfigMask) | _BV(REFS0) | _BV(ADLAR);
  ADCSRA = (ADCSRA & ~kAdcPrescalerMask) | _BV(ADPS2);
  ADMUX = (ADMUX & ~kAdcChannelMask) | kLineSensorsAdcChannel;
}

uint8_t AvrLineSensorBus::read(uint8_t channel) {
  PORTC = (PORTC & ~kMuxSelectMask) | (channel & kMuxSelectMask);
  return convert();
}

uint8_t AvrLineSensorBus::convert() {
  ADCSRA |= _BV(ADEN) | _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) {
  }
  ADCSRA &= ~_BV(ADEN);
  return ADCH;
}

}
