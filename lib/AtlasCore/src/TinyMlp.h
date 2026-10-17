/*
  TinyMlp.h - Red neuronal de una capa oculta en punto fijo para el AVR.

  Pesos en la flash (PROGMEM): int8 en la capa oculta, int16 en la de salida
  (son pocos y así la salida sale directo en las unidades deseadas). Entradas
  int8, capa oculta int16 con ReLU. Sin float ni memoria dinámica: unas 100
  MAC tardan ~0.1 ms en el ATmega328P.

  Escalas (las fija la herramienta de entrenamiento al cuantizar):
    oculta[j] = ReLU((sesgo1[j] + Σ w1[j][i]·entrada[i]) >> shift1)
    salida[k] = (sesgo2[k] + Σ w2[k][j]·oculta[j]) >> shift2
*/
#pragma once

#include <stdint.h>

#include "Progmem.h"

namespace atlas {

struct TinyMlpWeights {
  uint8_t inputs;
  uint8_t hidden;
  uint8_t outputs;
  const int8_t* weights1;  // [hidden][inputs]
  const int32_t* bias1;    // [hidden]
  uint8_t shift1;
  const int16_t* weights2;  // [outputs][hidden]
  const int32_t* bias2;     // [outputs]
  uint8_t shift2;
};

class TinyMlp {
 public:
  static constexpr uint8_t kMaxHidden = 16;

  explicit TinyMlp(const TinyMlpWeights& weights) : weights_(weights) {}

  // input: weights.inputs valores; output: weights.outputs valores.
  // Una sola copia en la flash aunque se llame desde varios lugares.
  __attribute__((noinline)) void evaluate(const int8_t* input, int32_t* output) const {
    int16_t hidden[kMaxHidden];
    const TinyMlpWeights& w = weights_;

    for (uint8_t j = 0; j < w.hidden; j++) {
      int32_t sum = readProgmemInt32(&w.bias1[j]);
      const int8_t* row = w.weights1 + static_cast<uint16_t>(j) * w.inputs;
      for (uint8_t i = 0; i < w.inputs; i++) sum += multiply8(readProgmemInt8(&row[i]), input[i]);
      sum >>= w.shift1;
      if (sum < 0) sum = 0;
      if (sum > 32767) sum = 32767;
      hidden[j] = static_cast<int16_t>(sum);
    }

    for (uint8_t k = 0; k < w.outputs; k++) {
      int32_t sum = readProgmemInt32(&w.bias2[k]);
      const int16_t* row = w.weights2 + static_cast<uint16_t>(k) * w.hidden;
      for (uint8_t j = 0; j < w.hidden; j++) {
        sum += static_cast<int32_t>(readProgmemInt16(&row[j])) * hidden[j];
      }
      output[k] = sum >> w.shift2;
    }
  }

 private:
  // int8 · int8 cabe en int16: en el AVR es una sola instrucción MULS en vez
  // de la rutina de 32 bits (~65 ciclos) que el compilador usaría si viera
  // que el producto se suma a un int32.
  static int16_t multiply8(int8_t a, int8_t b) {
#ifdef __AVR__
    int16_t product;
    asm("muls %1, %2\n\tmovw %0, r0\n\tclr __zero_reg__" : "=r"(product) : "d"(a), "d"(b));
    return product;
#else
    return static_cast<int16_t>(a * b);
#endif
  }

  const TinyMlpWeights& weights_;
};

}  // namespace atlas
