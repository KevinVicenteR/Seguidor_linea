/*
  NeuralHierarchy.h - Red jerárquica de control: una neurona principal
  (compuerta) reconoce la situación y cuatro subneuronas expertas proponen el
  % de kp y kd para cada una. El resultado es la mezcla ponderada:

      % = Σ peso_k · experto_k / Σ peso_k,   peso_k = ReLU(principal_k)

  Situaciones: 0 recta, 1 curva, 2 cruce o hueco, 3 recién recuperada la línea.
  La normalización por suma de ReLU (en vez de softmax) evita que la compuerta
  deje expertos sin usar (MPC-Net, Carius et al. 2019).
*/
#pragma once

#include <stdint.h>

#include "TinyMlp.h"

namespace atlas {

constexpr uint8_t kSituations = 4;
constexpr uint8_t kHierarchyInputs = 8;
constexpr uint8_t kHierarchyMinPercent = 70;
constexpr uint8_t kHierarchyMaxPercent = 130;

struct NeuralHierarchy {
  TinyMlpWeights gate;
  TinyMlpWeights experts[kSituations];
};

// input: kHierarchyInputs valores. weights (opcional): peso de cada situación
// en 0 ... 255 (para registros y entrenamiento).
void evaluateHierarchy(const NeuralHierarchy& hierarchy, const int8_t* input, uint8_t* kpPercent,
                       uint8_t* kdPercent, uint8_t* weights = nullptr);

}  // namespace atlas
