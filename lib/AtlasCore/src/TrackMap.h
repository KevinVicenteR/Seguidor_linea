/*
  TrackMap.h - Mapa de la pista aprendido en la vuelta de reconocimiento.

  La pista se divide en tramos de distancia estimada (sin encoders: se
  integra el PWM en el tiempo, ver DistanceEstimator). Cada tramo guarda qué
  tan cerrada es la curva (0 = recta, 255 = giro máximo o línea perdida) y se
  anotan los huecos, que sirven como referencias para re-sincronizar.
*/
#pragma once

#include <stdint.h>

namespace atlas {

constexpr uint16_t kMaxMapBins = 320;  // ~32 m con tramos de ~10 cm
constexpr uint8_t kMaxMapGaps = 8;

class TrackMap {
 public:
  void clear();
  void startRecording();
  // Acumula la curvatura del tramo actual; al cambiar de tramo se guarda el
  // promedio (el zigzag del control se cancela). Si en el tramo se perdió la
  // línea, se marca como curva máxima.
  void recordCurvature(uint16_t bin, uint8_t intensity, bool lineLost);
  void recordGap(uint16_t bin);
  // Termina la vuelta de reconocimiento; el mapa es válido si es lo bastante largo.
  void finishRecording(uint16_t lastBin, uint16_t minimumBins, uint16_t lostPerMille);

  bool isValid() const { return valid_; }
  uint16_t length() const { return length_; }
  uint8_t curvature(uint16_t bin) const { return bin < kMaxMapBins ? bins_[bin] : 255; }
  uint8_t gapCount() const { return gapCount_; }
  // Milésimas del reconocimiento que el robot pasó buscando la línea.
  uint16_t lostPerMille() const { return lostPerMille_; }
  uint16_t gap(uint8_t index) const { return gaps_[index]; }
  // Hueco del mapa más cercano a `bin` (dentro de `window` tramos), o -1.
  int32_t nearestGap(uint16_t bin, uint16_t window) const;

  // Acceso para guardar/cargar en memoria no volátil.
  uint8_t* rawBins() { return bins_; }
  uint16_t* rawGaps() { return gaps_; }
  void restore(uint16_t length, uint8_t gapCount, uint16_t lostPerMille);

 private:
  uint8_t bins_[kMaxMapBins] = {};
  uint16_t gaps_[kMaxMapGaps] = {};
  uint16_t length_ = 0;
  uint8_t gapCount_ = 0;
  uint16_t lostPerMille_ = 0;
  bool valid_ = false;

  void closeBin();
  uint16_t openBin_ = 0;
  uint32_t binSum_ = 0;
  uint16_t binSamples_ = 0;
  bool binLost_ = false;
};

}  // namespace atlas
