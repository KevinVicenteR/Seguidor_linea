#include <atomic>
#include <cstdio>
#include <functional>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "Report.h"
#include "ScenarioRunner.h"
#include "TrackLibrary.h"

#ifndef ATLAS_VIEWER_TEMPLATE
#define ATLAS_VIEWER_TEMPLATE "simulacion/visor/viewer_template.html"
#endif

using namespace sim;

namespace {

struct ExtremeCase {
  std::string name;
  std::function<void(RobotParams&, SensorModel&, ScenarioSpec&)> apply;
};

std::vector<ExtremeCase> extremeCases() {
  return {
      {"Nominal", [](RobotParams&, SensorModel&, ScenarioSpec&) {}},
      {"Motores lentos (90 ms)", [](RobotParams& r, SensorModel&, ScenarioSpec&) { r.motorTimeConstant = 0.090; }},
      {"Retraso de control 8 ms", [](RobotParams& r, SensorModel&, ScenarioSpec&) { r.commandLatencyS = 0.008; }},
      {"Inercia de giro alta", [](RobotParams& r, SensorModel&, ScenarioSpec&) { r.yawTimeConstant = 0.050; }},
      {"Robot real (lento + inercia + retraso)",
       [](RobotParams& r, SensorModel&, ScenarioSpec&) {
         r.motorTimeConstant = 0.070;
         r.yawTimeConstant = 0.040;
         r.commandLatencyS = 0.004;
       }},
      {"Zona muerta de motor (PWM 40)", [](RobotParams& r, SensorModel&, ScenarioSpec&) { r.motorDeadbandPwm = 40; }},
      {"Motor derecho 15% más débil", [](RobotParams& r, SensorModel&, ScenarioSpec&) { r.rightMotorGain = 0.85; }},
      {"Batería baja (-30% velocidad)", [](RobotParams& r, SensorModel&, ScenarioSpec&) { r.maxWheelSpeed *= 0.70; }},
      {"Batería 3S (+45% velocidad)", [](RobotParams& r, SensorModel&, ScenarioSpec&) { r.maxWheelSpeed *= 1.45; }},
      {"Llantas sucias (agarre 0.6)", [](RobotParams& r, SensorModel&, ScenarioSpec&) { r.tireFriction = 0.6; }},
      {"Sin succión", [](RobotParams&, SensorModel&, ScenarioSpec& s) { s.impellerPwm = 0; }},
      {"Sensor 6 dañado (siempre blanco)",
       [](RobotParams&, SensorModel& m, ScenarioSpec&) {
         m.deadSensor = 6;
         m.deadSensorValue = 220;
       }},
      {"Sensor 9 pegado (siempre negro)",
       [](RobotParams&, SensorModel& m, ScenarioSpec&) {
         m.deadSensor = 9;
         m.deadSensorValue = 25;
       }},
      {"Línea gastada (40% de contraste)", [](RobotParams&, SensorModel& m, ScenarioSpec&) { m.contrast = 0.40; }},
      {"Luz ambiental + ruido x3",
       [](RobotParams&, SensorModel& m, ScenarioSpec& s) {
         m.ambientOffset = 45;
         s.sensorNoiseSigma = 12.0;
       }},
      {"Sensores altos (lectura borrosa)", [](RobotParams&, SensorModel& m, ScenarioSpec&) { m.spotRadius = 0.008; }},
  };
}

struct FirmwareVariant {
  std::string name;
  atlas::RobotConfig config;
};

atlas::RobotConfig originalFirmware() {
  atlas::RobotConfig c;
  c.control.kp = 0.05f;
  c.control.kd = 0.35f;
  c.control.derivativeFilterShift = 0;
  c.control.curveSlowdownPercent = 0;
  c.control.maxForwardPwm = 200;
  c.lostLine.outerMotorPwm = 200;
  c.lostLine.gapBridgeMs = 0;
  c.sensors.trackNearestCluster = false;
  c.sensors.requiredCalibratedSensors = 0;
  c.adaptive.autoDamping = false;
  c.adaptive.speedGovernor = false;
  c.corner.enabled = false;
  return c;
}

struct Job {
  std::size_t variant;
  std::size_t caseIndex;
  std::size_t track;
  int16_t pwm;
  uint32_t seed;
};

}

int main(int argc, char** argv) {
  std::string outputDir = "resultados_simulacion";
  for (int i = 1; i + 1 < argc; i++) {
    if (std::string(argv[i]) == "--out") outputDir = argv[i + 1];
  }

  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  const std::vector<ExtremeCase> cases = extremeCases();
  const std::vector<std::string> trackNames = {"Pista de competencia", "Gran Premio", "Laberinto de esquinas vivas",
                                               "Chicanas largas"};

  const std::vector<FirmwareVariant> variants = {{"Original", originalFirmware()}, {"Nuevo", atlas::RobotConfig()}};

  std::vector<Job> jobs;
  for (std::size_t v = 0; v < variants.size(); v++)
  for (std::size_t c = 0; c < cases.size(); c++)
    for (std::size_t t = 0; t < tracks.size(); t++) {
      bool selected = false;
      for (const std::string& name : trackNames) selected = selected || tracks[t].track.name() == name;
      if (!selected) continue;
      for (int16_t pwm : {100, 160, 220})
        for (uint32_t seed : {7u, 8u}) jobs.push_back({v, c, t, pwm, seed * 1000 + static_cast<uint32_t>(c * 37 + t)});
    }

  std::vector<ScenarioResult> results(jobs.size());
  std::atomic<std::size_t> next{0};
  auto worker = [&]() {
    for (std::size_t i = next++; i < jobs.size(); i = next++) {
      const Job& job = jobs[i];
      RobotParams robot;
      SensorModel sensors;
      ScenarioSpec spec;
      spec.trackIndex = job.track;
      spec.cruisePwm = job.pwm;
      spec.seed = job.seed;
      cases[job.caseIndex].apply(robot, sensors, spec);
      spec.label = tracks[job.track].track.name() + " · " + variants[job.variant].name + ": " +
                   cases[job.caseIndex].name + " · PWM " + std::to_string(job.pwm);
      results[i] = ScenarioRunner(robot, sensors, variants[job.variant].config).run(tracks[job.track], spec);
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < std::max(1u, std::thread::hardware_concurrency()); t++) pool.emplace_back(worker);
  for (std::thread& th : pool) th.join();

  struct Summary {
    int runs = 0, failures = 0, passed = 0;
    double zigzag = 0, p95 = 0;
  };
  auto summarize = [&](std::size_t v, std::size_t c) {
    Summary s;
    for (std::size_t i = 0; i < jobs.size(); i++) {
      if (jobs[i].variant != v || jobs[i].caseIndex != c) continue;
      s.runs++;
      if (!results[i].passed()) {
        s.failures++;
        continue;
      }
      s.passed++;
      s.zigzag += results[i].zigzagPerMeter;
      s.p95 += results[i].p95OffsetMm;
    }
    if (s.passed) {
      s.zigzag /= s.passed;
      s.p95 /= s.passed;
    }
    return s;
  };

  std::printf("%-40s %17s %17s %19s\n", "Caso extremo", "Fallas orig->nuevo", "Zigzag/m o->n", "p95 mm o->n");
  int totals[2] = {0, 0};
  for (std::size_t c = 0; c < cases.size(); c++) {
    const Summary before = summarize(0, c);
    const Summary after = summarize(1, c);
    totals[0] += before.failures;
    totals[1] += after.failures;
    std::printf("%-40s %6d/%d -> %2d/%d %7.1f -> %5.1f %8.0f -> %5.0f\n", cases[c].name.c_str(), before.failures,
                before.runs, after.failures, after.runs, before.zigzag, after.zigzag, before.p95, after.p95);
  }
  std::printf("\nTOTAL fallas: original %d -> nuevo %d (de %zu corridas cada uno)\n", totals[0], totals[1],
              jobs.size() / variants.size());

  mkdir(outputDir.c_str(), 0755);
  const RobotParams robot;
  const ReportContext context{tracks, results, variants[1].config, robot};
  writeViewer(ATLAS_VIEWER_TEMPLATE, outputDir + "/extremos.html", context);
  std::printf("Visor: %s/extremos.html\n", outputDir.c_str());
  return 0;
}
