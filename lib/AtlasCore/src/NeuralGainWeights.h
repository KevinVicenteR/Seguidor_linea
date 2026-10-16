/*
  NeuralGainWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp
  (atlas_train_nn control). No editar a mano.

  Red 6 -> 8 ReLU -> 2: % de kp y % de kd en marcha.
  Entrenada el 2026-10-16 por evolución en el simulador.
  Validación (99 competencias de 3 intentos, semillas nuevas):
    sin red de control: salidas 0, tiempo medio 12.16 s, p95 39 mm, zigzag 1.25/m
    con red de control: salidas 0, tiempo medio 11.92 s, p95 38 mm, zigzag 1.14/m
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
    12, -12, 21, 22, -2, -7, 
    -22, -1, 20, -7, -2, 26, 
    42, 16, -63, -16, -25, 15, 
    9, 13, -3, -9, 27, -16, 
    32, 0, -15, 19, -13, 46, 
    4, 8, -2, -9, 12, -13, 
    -6, -7, -16, 34, -42, -35, 
    10, 38, 3, -13, 15, 4};
ATLAS_PROGMEM const int32_t kGainBias1[kGainHidden] = {
    2981, 917, 1692, 2077, 
    1849, -848, 2547, -91};
ATLAS_PROGMEM const int16_t kGainWeights2[kGainOutputs * kGainHidden] = {
    17, 2, -19, -9, 4, -15, -8, 0, 
    11, 25, 44, 16, 36, -15, 17, -11};
ATLAS_PROGMEM const int32_t kGainBias2[kGainOutputs] = {
    27004, 31891};

const TinyMlpWeights kGainNet = {kGainInputs, kGainHidden, kGainOutputs,
                                 kGainWeights1, kGainBias1, kGainShift1,
                                 kGainWeights2, kGainBias2, kGainShift2};

}  // namespace neural
}  // namespace atlas
