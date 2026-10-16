/*
  NeuralGainWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp
  (atlas_train_nn control). No editar a mano.

  Red 6 -> 8 ReLU -> 2: % de kp y % de kd en marcha.
  Entrenada el 2026-10-15 por evolución en el simulador.
  Validación (99 competencias de 3 intentos, semillas nuevas):
    sin red de control: salidas 0, tiempo medio 11.86 s, p95 38 mm, zigzag 1.61/m
    con red de control: salidas 0, tiempo medio 11.68 s, p95 36 mm, zigzag 1.38/m
*/
#pragma once

#include <stdint.h>

#include "Progmem.h"
#include "TinyMlp.h"

namespace atlas {
namespace neural {

constexpr uint8_t kGainInputs = 6;
constexpr uint8_t kGainHidden = 8;
constexpr uint8_t kGainOutputs = 2;  // % de kp, % de kd
constexpr uint8_t kGainShift1 = 7;
constexpr uint8_t kGainShift2 = 8;

ATLAS_PROGMEM const int8_t kGainWeights1[kGainHidden * kGainInputs] = {
    0, 0, 6, 16, 33, 0, 
    22, -2, 34, 33, 34, 13, 
    -4, -8, 17, 3, 40, -15, 
    3, -57, 26, 8, -37, -19, 
    16, -34, -1, 26, 3, 51, 
    -45, -1, 21, -4, -17, -6, 
    -30, -25, -31, 22, 10, 0, 
    -26, -6, -15, 13, -28, -36};
ATLAS_PROGMEM const int32_t kGainBias1[kGainHidden] = {
    65, 1371, 617, -935, 
    242, 2229, -1997, 1547};
ATLAS_PROGMEM const int16_t kGainWeights2[kGainOutputs * kGainHidden] = {
    4, 16, 9, 8, 9, 2, -15, 1, 
    21, 35, 5, 0, 33, 4, 1, -10};
ATLAS_PROGMEM const int32_t kGainBias2[kGainOutputs] = {
    24415, 32286};

const TinyMlpWeights kGainNet = {kGainInputs, kGainHidden, kGainOutputs,
                                 kGainWeights1, kGainBias1, kGainShift1,
                                 kGainWeights2, kGainBias2, kGainShift2};

}  // namespace neural
}  // namespace atlas
