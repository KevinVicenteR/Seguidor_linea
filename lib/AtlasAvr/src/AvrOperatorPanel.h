/*
  AvrOperatorPanel.h - Botones SW1 (PC5) / SW2 (PD7), módulo de arranque
  READY (PD3) / GO (PD4) y LEDs LED0 (PD2), LED1 (PB0), LED2 (PB5).
*/
#pragma once

#include <Arduino.h>

#include "Hal.h"

namespace atlas {

class AvrOperatorPanel : public IOperatorInput, public IIndicators {
 public:
  void begin() {
    DDRD |= _BV(DDD2);
    DDRB |= _BV(DDB0) | _BV(DDB5);
    PORTC |= _BV(PORTC5);  // pull-up SW1
    PORTD |= _BV(PORTD7);  // pull-up SW2
  }

  bool button1() override { return !(PINC & _BV(PINC5)); }
  bool button2() override { return !(PIND & _BV(PIND7)); }
  bool startReady() override { return PIND & _BV(PIND3); }
  bool startGo() override { return PIND & _BV(PIND4); }

  void setLed(Led led, bool on) override {
    switch (led) {
      case Led::Led0:
        writeBit(PORTD, PORTD2, on);
        break;
      case Led::Led1:
        writeBit(PORTB, PORTB0, on);
        break;
      case Led::Led2:
        writeBit(PORTB, PORTB5, on);
        break;
    }
  }

 private:
  static void writeBit(volatile uint8_t& port, uint8_t bit, bool on) {
    port = (port & ~_BV(bit)) | (static_cast<uint8_t>(on) << bit);
  }
};

}  // namespace atlas
