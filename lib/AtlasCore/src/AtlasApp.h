/*
  AtlasApp.h - Máquina de estados de la secuencia de operación:

    Boot -> WaitCalibration -> Calibrating -> Ready <-> Running
            (SW1 o SW2: línea negra)    |      |  GO     quitar GO
                    ^-------------------+      |
                    calibración incompleta     +-- SW1 + SW2 juntos: recalibrar

  - Calibración válida: animación de confirmación y LED0 + LED1 fijos. Si
    hasta 2 sensores no calibraron (dañados), antes LED2 da 3 destellos lentos.
  - Aprendizaje de pista: sin mapa, la carrera es de reconocimiento con el
    perfil Seguro y graba la pista. Con mapa (LED0 y LED1 parpadean
    alternados), la carrera usa el perfil Rápido y la velocidad del mapa.
    Recalibrar borra el mapa. Si el reconocimiento falla (más de 1.5 s sin
    línea: robot fuera de la pista) el mapa se descarta (LED2 da 4 destellos rápidos); si falla una
    carrera guiada, el siguiente intento sale más prudente.
  - Con memoria no volátil, la calibración, el mapa y la velocidad elegida se
    guardan; al encender se cargan (LED2 da 2 destellos) y el robot queda
    listo sin recalibrar ni volver a ajustar la velocidad.
  - Quitar GO frena y regresa a Ready conservando calibración y velocidad:
    se puede volver a dar READY/GO sin recalibrar. RESET o apagar conservan
    calibración y mapa (EEPROM); SW1 + SW2 juntos los borran y recalibran.

  tick() se llama continuamente desde loop(). Las animaciones cortas siguen
  siendo bloqueantes (igual que el original) a través de IClock::delayMs.
*/
#pragma once

#include <stdint.h>

#include "Config.h"
#include "CalibrationStore.h"
#include "Hal.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "RunLog.h"
#include "SpeedSettingStore.h"
#include "TrackMap.h"
#include "TrackMapStore.h"
#include "Ramp.h"
#include "StatusLeds.h"

namespace atlas {

// Piezas opcionales de la aplicación.
struct AppOptions {
  IDiagnostics* diagnostics = nullptr;     // modo de diagnóstico (solo hardware real)
  const RobotConfig* fastConfig = nullptr;  // perfil Rápido para las carreras con mapa
  IPersistentStorage* storage = nullptr;    // guardar calibración y mapa al apagar
  TrackMap* trackMap = nullptr;             // aprendizaje de pista
};

enum class AppState : uint8_t { Boot, WaitCalibration, Calibrating, Ready, Running };

class AtlasApp {
 public:
  AtlasApp(IClock& clock, IOperatorInput& input, IIndicators& indicators, LineSensorArray& sensors,
           IDrivetrain& drivetrain, LineFollower& follower, const RobotConfig& config,
           const AppOptions& options = AppOptions());

  void begin();
  void tick();

  AppState state() const { return state_; }
  int16_t cruisePwm() const { return cruisePwm_; }
  uint16_t racesCompleted() const { return racesCompleted_; }
  // La próxima carrera será guiada por el mapa aprendido (perfil Rápido).
  bool hasTrackMap() const { return trackMap_ != nullptr && trackMap_->isValid(); }

 private:
  bool anyButton() { return input_.button1() || input_.button2(); }

  void waitCalibration();
  void calibrate();
  void prepareToRun();
  void run();

  bool loadSavedCalibration();
  void loadSavedSpeed();
  void saveSpeed();
  void forgetTrackMap();
  void saveTrackMapIfLearned();
  void logRace();
  void startCalibration();
  void finishCalibration();
  void updateReadyIndicator();
  void updateImpellerPreSpin();
  void adjustCruiseSpeed();
  void startRace();
  void finishRace();
  bool recalibrationRequested();
  bool lapResetRequested();
  void returnToFirstLap();

  IClock& clock_;
  IOperatorInput& input_;
  StatusLeds leds_;
  LineSensorArray& sensors_;
  IDrivetrain& drivetrain_;
  LineFollower& follower_;
  const RobotConfig& config_;
  IDiagnostics* diagnostics_;
  const RobotConfig* fastConfig_;
  IPersistentStorage* storage_;
  TrackMap* trackMap_;

  AppState state_ = AppState::Boot;
  int16_t cruisePwm_;
  bool calibrationBlink_ = true;
  bool preSpinActive_ = false;
  LinearRamp preSpinRamp_;
  uint32_t lastControlUs_ = 0;
  uint16_t racesCompleted_ = 0;
  uint32_t raceStartMs_ = 0;
  uint32_t raceStopMs_ = 0;
  // Gesto del control remoto: pulsos de READY sin GO para volver a la vuelta 1.
  bool readyWasOn_ = false;
  uint8_t readyPulses_ = 0;
  uint32_t firstReadyPulseMs_ = 0;
};

}  // namespace atlas
