/*
  NeuralGainWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp
  (atlas_train_nn control). No editar a mano.

  Red 6 -> 8 ReLU -> 2: % de kp y % de kd en marcha.
  Entrenada el 2026-10-15 por evolución en el simulador.
  Validación (99 competencias de 3 intentos, semillas nuevas):
    sin red de control: salidas 0, tiempo medio 11.87 s, p95 38 mm, zigzag 1.6/m
    con red de control: salidas 0, tiempo medio 11.72 s, p95 37 mm, zigzag 1.32/m
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
    -7, -3, 8, 16, 25, 8, 
    21, 2, 25, 24, 29, 0, 
    -5, -14, 18, 0, 39, -20, 
    6, -58, 21, 8, -34, -23, 
    19, -33, 7, 31, 2, 47, 
    -40, 0, 22, -14, -18, -4, 
    -25, -18, -30, 15, -4, 4, 
    -17, -1, -14, 12, -34, -37};
ATLAS_PROGMEM const int32_t kGainBias1[kGainHidden] = {
    214, 961, 398, -2023, 
    -237, 3351, -1342, 1971};
ATLAS_PROGMEM const int16_t kGainWeights2[kGainOutputs * kGainHidden] = {
    2, 8, 7, 9, -3, 2, -10, 2, 
    31, 40, 5, 1, 39, 4, 2, -13};
ATLAS_PROGMEM const int32_t kGainBias2[kGainOutputs] = {
    24104, 31338};

const TinyMlpWeights kGainNet = {kGainInputs, kGainHidden, kGainOutputs,
                                 kGainWeights1, kGainBias1, kGainShift1,
                                 kGainWeights2, kGainBias2, kGainShift2};

}  // namespace neural
}  // namespace atlas
