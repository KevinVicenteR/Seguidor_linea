#include <atomic>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "FirmwareConfig.h"
#include "Report.h"
#include "ScenarioRunner.h"
#include "TrackLibrary.h"

#ifndef ATLAS_ANALYZER_TEMPLATE
#define ATLAS_ANALYZER_TEMPLATE "simulacion/visor/analyzer_template.html"
#endif

#ifndef ATLAS_VIEWER_TEMPLATE
#define ATLAS_VIEWER_TEMPLATE "simulacion/visor/viewer_template.html"
#endif

using namespace sim;

namespace {

struct Options {
  std::string outputDir = "resultados_simulacion";
  std::string templatePath = ATLAS_VIEWER_TEMPLATE;
  int onlyTrack = -1;
  bool quick = false;
  bool firmware = false;
  bool neuralNetwork = true;
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    if (arg == "--out" && i + 1 < argc) options.outputDir = argv[++i];
    else if (arg == "--template" && i + 1 < argc) options.templatePath = argv[++i];
    else if (arg == "--track" && i + 1 < argc) options.onlyTrack = std::stoi(argv[++i]);
    else if (arg == "--firmware") options.firmware = true;
    else if (arg == "--quick") options.quick = true;
    else if (arg == "--sin-red") options.neuralNetwork = false;
  }
  return options;
}

bool isBeyondGeometry(const std::string& name) { return name == "Trébol de lazos" || name == "Triángulo de rizos"; }

ScenarioSpec makeSpec(std::size_t track, const std::string& trackName, int16_t pwm, bool required,
                      const std::string& suffix = "") {
  ScenarioSpec spec;
  spec.trackIndex = track;
  spec.cruisePwm = pwm;
  spec.required = required;
  spec.seed = static_cast<uint32_t>(1000 + track * 31 + pwm);
  spec.label = trackName + " · PWM " + std::to_string(pwm) + suffix;
  return spec;
}

std::size_t findTrack(const std::vector<TrackEntry>& tracks, const std::string& name) {
  for (std::size_t i = 0; i < tracks.size(); i++) {
    if (tracks[i].track.name() == name) return i;
  }
  return 0;
}

std::vector<ScenarioSpec> buildScenarios(const std::vector<TrackEntry>& tracks, const Options& options) {
  if (options.firmware) {
    std::vector<ScenarioSpec> specs;
    const auto safe = atlas::firmwareSafeConfig(options.neuralNetwork);
    const auto fast = atlas::firmwareFastConfig(options.neuralNetwork);
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      if (options.onlyTrack >= 0 && static_cast<int>(i) != options.onlyTrack) continue;
      const auto& name = tracks[i].track.name();
      ScenarioSpec spec = makeSpec(i, name, safe.speed.basePwm, !isBeyondGeometry(name), " · firmware");
      spec.impellerPwm = safe.impeller.pwm;
      spec.fastImpellerPwm = fast.impeller.pwm;
      spec.learnTrack = true;
      spec.attempts = 3;
      specs.push_back(spec);
    }
    return specs;
  }
  const std::vector<int16_t> requiredSpeeds = options.quick ? std::vector<int16_t>{190} : std::vector<int16_t>{130, 160, 190, 220};
  std::vector<ScenarioSpec> specs;

  for (std::size_t i = 0; i < tracks.size(); i++) {
    if (options.onlyTrack >= 0 && static_cast<int>(i) != options.onlyTrack) continue;
    const std::string& name = tracks[i].track.name();
    const bool beyondGeometry = isBeyondGeometry(name);
    for (int16_t pwm : requiredSpeeds) {
      specs.push_back(makeSpec(i, name, pwm, !beyondGeometry, beyondGeometry ? " (lazos < 18 cm)" : ""));
    }
    if (!options.quick) specs.push_back(makeSpec(i, name, 250, false, " (extremo)"));
  }
  if (options.quick || options.onlyTrack >= 0) return specs;

  const std::size_t competition = findTrack(tracks, "Pista de competencia");
  for (int16_t pwm : {190, 220}) {
    ScenarioSpec fromGap = makeSpec(competition, tracks[competition].track.name(), pwm, true, " · sale del hueco");
    fromGap.customStart = true;
    fromGap.startPoint = competitionGapCenter();
    specs.push_back(fromGap);
  }

  for (std::size_t i = 0; i < tracks.size(); i++) {
    if (tracks[i].track.name() == "Trébol de lazos") continue;
    ScenarioSpec learning =
        makeSpec(i, tracks[i].track.name(), 100, !isBeyondGeometry(tracks[i].track.name()), " · aprendizaje");
    learning.learnTrack = true;
    learning.attempts = 3;
    learning.fastImpellerPwm = 215;
    specs.push_back(learning);
  }

  const std::size_t star = findTrack(tracks, "Estrella de cinco puntas");
  ScenarioSpec noisy = makeSpec(star, tracks[star].track.name(), 190, true, " · ruido x3");
  noisy.sensorNoiseSigma = 12.0;
  specs.push_back(noisy);

  const std::size_t grandPrix = findTrack(tracks, "Gran Premio");
  ScenarioSpec noFan = makeSpec(grandPrix, tracks[grandPrix].track.name(), 190, false, " · sin impeller");
  noFan.impellerPwm = 0;
  specs.push_back(noFan);

  return specs;
}

std::string exampleRobotLog(const std::vector<TrackEntry>& tracks) {
  const std::size_t competition = findTrack(tracks, "Pista de competencia");
  atlas::RobotConfig aggressive = atlas::fastProfile();
  aggressive.planner.fullTurnPwm = 180;
  aggressive.planner.brakePwmPerBin = 30;
  aggressive.planner.lookaheadBins = 4;
  aggressive.planner.maxPwm = 255;
  aggressive.planner.slowdownPerLostPerMille = 0;
  aggressive.adaptive.speedGovernor = false;
  RobotParams slowRobot;
  slowRobot.motorTimeConstant = 0.070;
  slowRobot.yawTimeConstant = 0.040;
  slowRobot.commandLatencyS = 0.004;

  ScenarioSpec spec;
  spec.trackIndex = competition;
  spec.seed = 505;
  spec.learnTrack = true;
  spec.attempts = 3;
  spec.impellerPwm = 0;
  const std::vector<ScenarioResult> attempts =
      ScenarioRunner(slowRobot, SensorModel(), atlas::safeProfile(), aggressive).runAttempts(tracks[competition], spec);
  return attempts.back().robotLog;
}

std::vector<ScenarioResult> runAll(const std::vector<TrackEntry>& tracks, const std::vector<ScenarioSpec>& specs,
                                   bool neuralNetwork, bool firmware) {
  std::vector<std::vector<ScenarioResult>> perSpec(specs.size());
  std::atomic<std::size_t> next{0};
  std::atomic<std::size_t> done{0};
  atlas::RobotConfig safe = firmware ? atlas::firmwareSafeConfig() : atlas::safeProfile();
  atlas::RobotConfig fast = firmware ? atlas::firmwareFastConfig() : atlas::fastProfile();
  atlas::useNeuralNetworks(safe, neuralNetwork);
  atlas::useNeuralNetworks(fast, neuralNetwork);
  const ScenarioRunner runner(RobotParams(), SensorModel(), safe, fast);

  auto worker = [&]() {
    for (std::size_t i = next++; i < specs.size(); i = next++) {
      perSpec[i] = runner.runAttempts(tracks[specs[i].trackIndex], specs[i]);
      for (std::size_t a = 0; perSpec[i].size() > 1 && a < perSpec[i].size(); a++) {
        perSpec[i][a].spec.label += a == 0 ? " · Intento 1 (reconocimiento)"
                                           : " · Intento " + std::to_string(a + 1) + " (con mapa)";
      }
      std::fprintf(stderr, "\r  simulando... %zu/%zu", ++done, specs.size());
    }
  };

  const unsigned threads = std::max(1u, std::thread::hardware_concurrency());
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < threads; t++) pool.emplace_back(worker);
  for (std::thread& thread : pool) thread.join();
  std::fprintf(stderr, "\n");

  std::vector<ScenarioResult> results;
  for (const std::vector<ScenarioResult>& attempts : perSpec) results.insert(results.end(), attempts.begin(), attempts.end());
  return results;
}

void printSummary(const std::vector<ScenarioResult>& results) {
  std::printf("\n%-48s %6s %8s %8s %8s %8s %8s %7s  %s\n", "Escenario", "Estado", "Tiempo", "v media", "v máx",
              "err p95", "err máx", "perdida", "Falla");
  std::printf("%s\n", std::string(118, '-').c_str());
  for (const ScenarioResult& r : results) {
    const char* status = r.passed() ? "OK" : (r.spec.required ? "FALLA" : "falla*");
    std::printf("%-48s %6s %7.2fs %6.2fm/s %6.2fm/s %6.1fmm %6.0fmm %6.1f%%  %s\n", r.spec.label.c_str(), status,
                r.raceTime, r.averageSpeed, r.maxSpeed, r.p95OffsetMm, r.maxOffsetMm, r.lostLinePercent,
                r.failureReason.c_str());
  }
}

}

int main(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  const std::vector<ScenarioSpec> specs = buildScenarios(tracks, options);

  std::printf("ATLAS · banco de simulación: %zu pistas, %zu escenarios\n", tracks.size(), specs.size());
  const std::vector<ScenarioResult> results = runAll(tracks, specs, options.neuralNetwork, options.firmware);
  printSummary(results);

  mkdir(options.outputDir.c_str(), 0755);
  atlas::RobotConfig firmware = options.firmware ? atlas::firmwareSafeConfig() : atlas::safeProfile();
  atlas::useNeuralNetworks(firmware, options.neuralNetwork);
  const RobotParams robot;
  const ReportContext context{tracks, results, firmware, robot};
  writeViewer(options.templatePath, options.outputDir + "/index.html", context);
  writeSummaryCsv(options.outputDir + "/resumen.csv", context);
  writeAnalyzer(ATLAS_ANALYZER_TEMPLATE, options.outputDir + "/analizador.html", tracks, exampleRobotLog(tracks));

  int requiredFailures = 0;
  int passed = 0;
  for (const ScenarioResult& r : results) {
    if (r.passed()) passed++;
    else if (r.spec.required) requiredFailures++;
  }
  std::printf("\nAprobados: %d/%zu   Obligatorios fallidos: %d   (* = prueba extrema, no obligatoria; límite 40 s por pista)\n", passed,
              results.size(), requiredFailures);
  std::printf("Visor: %s/index.html\n", options.outputDir.c_str());
  return requiredFailures == 0 ? 0 : 1;
}
