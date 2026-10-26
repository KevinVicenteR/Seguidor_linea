#pragma once

#include <stdint.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#define ATLAS_PROGMEM PROGMEM
#else
#define ATLAS_PROGMEM
#endif

namespace atlas {

inline int8_t readProgmemInt8(const int8_t* address) {
#ifdef __AVR__
  return static_cast<int8_t>(pgm_read_byte(address));
#else
  return *address;
#endif
}

inline int16_t readProgmemInt16(const int16_t* address) {
#ifdef __AVR__
  return static_cast<int16_t>(pgm_read_word(address));
#else
  return *address;
#endif
}

inline int32_t readProgmemInt32(const int32_t* address) {
#ifdef __AVR__
  return static_cast<int32_t>(pgm_read_dword(address));
#else
  return *address;
#endif
}

}
