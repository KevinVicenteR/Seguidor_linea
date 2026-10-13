/*
  StatusLeds.h - Patrones visuales de los LEDs (animaciones de arranque,
  confirmación y número binario) construidos sobre IIndicators.
*/
#pragma once

#include <stdint.h>

#include "Hal.h"

namespace atlas {

class StatusLeds {
 public:
  StatusLeds(IIndicators& indicators, IClock& clock) : indicators_(indicators), clock_(clock) {}

  void set(Led led, bool on) { indicators_.setLed(led, on); }

  void setAll(bool on) {
    set(Led::Led0, on);
    set(Led::Led1, on);
    set(Led::Led2, on);
  }

  // Muestra 0-7 en binario: LED0 = bit 2, LED2 = bit 1, LED1 = bit 0.
  void showNumber(uint8_t number) {
    const uint8_t value = number > 7 ? 0 : number;
    set(Led::Led0, value & 0x4);
    set(Led::Led2, value & 0x2);
    set(Led::Led1, value & 0x1);
  }

  void blink(Led led, uint16_t halfPeriodMs) {
    set(led, true);
    clock_.delayMs(halfPeriodMs);
    set(led, false);
    clock_.delayMs(halfPeriodMs);
  }

  void confirm(uint16_t halfPeriodMs, uint8_t cycles) {
    for (uint8_t i = 0; i < cycles; i++) {
      setAll(true);
      clock_.delayMs(halfPeriodMs);
      setAll(false);
      clock_.delayMs(halfPeriodMs);
    }
  }

  // LED0 y LED1 fijos: la calibración es válida y el robot espera READY/GO.
  void showCalibrated() {
    set(Led::Led0, true);
    set(Led::Led1, true);
  }

  // LED2 parpadeando rápido: calibración incompleta, hay que repetirla.
  void calibrationError() {
    setAll(false);
    for (uint8_t i = 0; i < 8; i++) blink(Led::Led2, 60);
  }

  void bootAnimation() {
    for (uint8_t i = 0; i < 10; i++) blink(Led::Led2, 25);
  }

 private:
  IIndicators& indicators_;
  IClock& clock_;
};

}  // namespace atlas
