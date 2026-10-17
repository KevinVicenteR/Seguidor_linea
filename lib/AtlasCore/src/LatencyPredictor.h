/*
  LatencyPredictor.h - Compensación del retraso del motor (predictor de Smith
  simplificado).

  Los motores tardan en seguir la corrección de dirección: parte de lo que el
  PD ya mandó todavía no hizo efecto. El predictor sigue la corrección con un
  filtro de primer orden (constante de 2^shift ciclos), que aproxima lo que
  los motores ya alcanzaron; la diferencia es la corrección "pendiente". El PD
  trabaja con el error menos lo que esa corrección pendiente todavía va a
  mover la línea, así no sigue empujando cuando el giro ya viene en camino.
*/
#pragma once

#include <stdint.h>

namespace atlas {

class LatencyPredictor {
 public:
  void reset() {
    realized16_ = 0;
    lastCorrection_ = 0;
  }
  void remember(int16_t correction) { lastCorrection_ = correction; }

  // Desplazamiento del error (unidades de posición) que causará la corrección
  // pendiente: gain/4 unidades por PWM pendiente.
  int16_t pendingErrorShift(uint8_t shift, int8_t gain) {
    const int32_t commanded16 = static_cast<int32_t>(lastCorrection_) * 16;
    realized16_ += (commanded16 - realized16_) >> shift;
    const int32_t pending = (commanded16 - realized16_) >> 4;
    return static_cast<int16_t>((pending * gain) >> 2);
  }

 private:
  int32_t realized16_ = 0;  // corrección que ya alcanzaron los motores, ×16
  int16_t lastCorrection_ = 0;
};

}  // namespace atlas
