/*
  NeuralDerivativeWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp
  (atlas_train_nn derivada). Marcador inicial: red sin entrenar, desactivada.
*/
#pragma once

#include <stdint.h>

#include "Progmem.h"
#include "TinyMlp.h"

namespace atlas {
namespace neural {

constexpr bool kDerivativeTrained = false;
constexpr uint8_t kDerivativeInputs = 7;
constexpr uint8_t kDerivativeHidden = 8;

ATLAS_PROGMEM const int8_t kDerivativeWeights1[kDerivativeHidden * kDerivativeInputs] = {};
ATLAS_PROGMEM const int32_t kDerivativeBias1[kDerivativeHidden] = {};
ATLAS_PROGMEM const int16_t kDerivativeWeights2[kDerivativeHidden] = {};
ATLAS_PROGMEM const int32_t kDerivativeBias2[1] = {};

const TinyMlpWeights kDerivativeNet = {kDerivativeInputs, kDerivativeHidden, 1,
                                       kDerivativeWeights1, kDerivativeBias1, 0,
                                       kDerivativeWeights2, kDerivativeBias2, 0};

}  // namespace neural
}  // namespace atlas
