#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "FirmwareConfig.h"
#include "ScenarioRunner.h"
#include "TrackLibrary.h"

using namespace sim;

namespace {

constexpr int kEvery = 4;

struct Frame {
  uint16_t ms;
  int16_t error, left, right;
  uint8_t flags;
  bool recovering() const { return flags & 1; }
  bool bridging() const { return flags & 2; }
  int mode() const { return (flags >> 2) & 3; }
};

std::vector<Frame> readLog(const std::string& path, int* rejected) {
  std::ifstream in(path, std::ios::binary);
  const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  std::vector<Frame> frames;
  *rejected = 0;
  auto i16 = [&](std::size_t at) { return static_cast<int16_t>(bytes[at] | (bytes[at + 1] << 8)); };
  for (std::size_t i = 0; i + 12 <= bytes.size();) {
    if (bytes[i] != 0xA5 || bytes[i + 1] != 0x5A) {
      i++;
      continue;
    }
    uint8_t check = 0;
    for (std::size_t k = 2; k < 11; k++) check ^= bytes[i + k];
    if (check != bytes[i + 11]) {
      (*rejected)++;
      i++;
      continue;
    }
    frames.push_back({static_cast<uint16_t>(i16(i + 2)), i16(i + 4), i16(i + 6), i16(i + 8), bytes[i + 10]});
    i += 12;
  }
  return frames;
}

void writeLog(const std::string& path, const std::vector<Frame>& frames) {
  std::ofstream out(path, std::ios::binary);
  for (const Frame& f : frames) {
    uint8_t b[12] = {0xA5, 0x5A, static_cast<uint8_t>(f.ms), static_cast<uint8_t>(f.ms >> 8),
                     static_cast<uint8_t>(f.error), static_cast<uint8_t>(f.error >> 8),
                     static_cast<uint8_t>(f.left), static_cast<uint8_t>(f.left >> 8),
                     static_cast<uint8_t>(f.right), static_cast<uint8_t>(f.right >> 8), f.flags, 0};
    for (int k = 2; k < 11; k++) b[11] ^= b[k];
    out.write(reinterpret_cast<const char*>(b), sizeof(b));
  }
}

struct Fingerprint {
  double oscillationsPerS = 0, errorRms = 0, steeringRms = 0, seconds = 0;
  double changeRms = 0;
  std::size_t frames = 0;
};

Fingerprint fingerprint(const std::vector<Frame>& frames, int mode) {
  Fingerprint f;
  double error2 = 0, steer2 = 0;
  int side = 0, crossings = 0;
  const Frame* previous = nullptr;
  double change2 = 0;
  std::size_t changes = 0;
  for (const Frame& fr : frames) {
    if (fr.mode() != mode || fr.recovering() || fr.bridging()) {
      previous = nullptr;
      side = 0;
      continue;
    }
    if (previous) {
      const double change = fr.error - previous->error;
      change2 += change * change;
      changes++;
    }
    if (previous) {
      const uint16_t dt = static_cast<uint16_t>(fr.ms - previous->ms);
      if (dt < 50) f.seconds += dt / 1000.0;
    }
    previous = &fr;
    error2 += static_cast<double>(fr.error) * fr.error;
    const double steering = (fr.left - fr.right) / 2.0;
    steer2 += steering * steering;
    const int now = fr.error > 300 ? 1 : (fr.error < -300 ? -1 : 0);
    if (now != 0 && side != 0 && now != side) crossings++;
    if (now != 0) side = now;
    f.frames++;
  }
  if (f.frames == 0) return f;
  f.errorRms = std::sqrt(error2 / f.frames);
  f.steeringRms = std::sqrt(steer2 / f.frames);
  f.oscillationsPerS = f.seconds > 0 ? crossings / 2.0 / f.seconds : 0;
  f.changeRms = changes ? std::sqrt(change2 / changes) : 0;
  return f;
}

struct Candidate {
  double motor, yaw, latency;
};

RobotParams robotFor(const Candidate& c) {
  RobotParams p;
  p.motorTimeConstant = c.motor;
  p.yawTimeConstant = c.yaw;
  p.commandLatencyS = c.latency;
  return p;
}

std::vector<Frame> simulate(const std::vector<TrackEntry>& tracks, std::size_t track, int pwm, int mode,
                            const Candidate& c, uint32_t seed) {
  const atlas::RobotConfig safe = atlas::firmwareSafeConfig(), fast = atlas::firmwareFastConfig();
  ScenarioRunner runner(robotFor(c), SensorModel(), safe, fast);
  std::vector<Frame> frames;
  uint32_t lastCycle = 0;
  runner.setObserver([&](double seconds, const atlas::LineSensorArray&, const atlas::LineFollower& follower,
                         const RobotModel&) {
    if (!follower.isRunning()) return;
    const uint32_t cycle = follower.controlCycles();
    if (cycle == lastCycle || cycle % kEvery != 0) return;
    lastCycle = cycle;
    const atlas::MotorCommand& command = follower.lastCommand();
    Frame f;
    f.ms = static_cast<uint16_t>(std::lround(seconds * 1000.0));
    f.error = static_cast<int16_t>(static_cast<int>(follower.lastReading().position) - atlas::kLinePositionCenter);
    f.left = command.left;
    f.right = command.right;
    f.flags = static_cast<uint8_t>((follower.isRecovering() ? 1 : 0) | (follower.isBridging() ? 2 : 0) |
                                   (static_cast<uint8_t>(follower.mode()) << 2));
    frames.push_back(f);
  });
  ScenarioSpec spec;
  spec.trackIndex = track;
  spec.cruisePwm = static_cast<int16_t>(pwm);
  spec.seed = seed;
  spec.learnTrack = mode != 0;
  spec.attempts = mode == 2 ? 2 : 1;
  spec.impellerPwm = safe.impeller.pwm;
  spec.fastImpellerPwm = fast.impeller.pwm;
  runner.runAttempts(tracks[track], spec);
  return frames;
}

double distance(const Fingerprint& a, const Fingerprint& b) {
  auto term = [](double x, double y) {
    const double r = std::log((x + 1e-3) / (y + 1e-3));
    return r * r;
  };
  return term(a.oscillationsPerS, b.oscillationsPerS) + term(a.errorRms, b.errorRms) +
         term(a.steeringRms, b.steeringRms) + term(a.changeRms, b.changeRms);
}

void printFingerprint(const char* label, const Fingerprint& f) {
  std::printf("  %-28s oscil. %.2f/s  error %5.0f (%.1f mm)  corrección %5.1f  cambio %5.1f\n", label,
              f.oscillationsPerS, f.errorRms, f.errorRms * 0.009, f.steeringRms, f.changeRms);
}

int fit(const std::vector<Frame>& log, std::size_t track, int pwm, const std::vector<TrackEntry>& tracks) {
  std::map<int, std::size_t> perMode;
  for (const Frame& f : log) perMode[f.mode()]++;
  int mode = 0;
  for (const auto& m : perMode)
    if (m.second > perMode[mode]) mode = m.first;
  const char* modes[] = {"normal", "reconocimiento", "con mapa", "?"};
  const Fingerprint target = fingerprint(log, mode);
  std::printf("Registro: %zu tramas; se usa el modo \"%s\" (%zu tramas útiles)\n", log.size(), modes[mode],
              target.frames);
  if (target.frames < 300) {
    std::printf("Muy pocas tramas útiles: haz una vuelta más larga.\n");
    return 1;
  }
  printFingerprint("robot real", target);

  std::vector<Candidate> grid;
  for (double motor : {0.025, 0.035, 0.045, 0.055, 0.07, 0.09})
    for (double yaw : {0.0, 0.01, 0.02, 0.03, 0.04, 0.06})
      for (double latency : {0.0, 0.002, 0.004, 0.006}) grid.push_back({motor, yaw, latency});
  const uint32_t seeds[] = {11, 23};
  std::vector<Fingerprint> results(grid.size());
  std::atomic<std::size_t> next{0};
  auto worker = [&] {
    for (std::size_t i = next++; i < grid.size(); i = next++) {
      Fingerprint sum;
      for (uint32_t seed : seeds) {
        const Fingerprint f = fingerprint(simulate(tracks, track, pwm, mode, grid[i], seed), mode);
        sum.oscillationsPerS += f.oscillationsPerS / 2;
        sum.errorRms += f.errorRms / 2;
        sum.steeringRms += f.steeringRms / 2;
        sum.seconds += f.seconds / 2;
        sum.changeRms += f.changeRms / 2;
      }
      results[i] = sum;
    }
  };
  std::vector<std::thread> pool;
  for (unsigned k = 0; k < std::max(2u, std::thread::hardware_concurrency()); k++) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();

  std::vector<std::size_t> order(grid.size());
  for (std::size_t i = 0; i < order.size(); i++) order[i] = i;
  std::sort(order.begin(), order.end(),
            [&](std::size_t a, std::size_t b) { return distance(results[a], target) < distance(results[b], target); });
  std::printf("\nPista \"%s\", PWM %d. Modelos más parecidos (de %zu):\n", tracks[track].track.name().c_str(), pwm,
              grid.size());
  for (std::size_t r = 0; r < 5; r++) {
    const Candidate& c = grid[order[r]];
    char label[64];
    std::snprintf(label, sizeof(label), "motor %.3f giro %.2f lat %.3f", c.motor, c.yaw, c.latency);
    std::printf("  dist %.3f", distance(results[order[r]], target));
    printFingerprint(label, results[order[r]]);
  }
  const Candidate& best = grid[order[0]];
  std::printf("\nPara el simulador (RobotParams):\n  motorTimeConstant = %.3f;\n  yawTimeConstant = %.3f;\n"
              "  commandLatencyS = %.3f;\n",
              best.motor, best.yaw, best.latency);
  std::printf("Referencia de los modelos usados: nominal 0.035/0/0, moderado 0.055/0.02/0.003, real 0.07/0.04/0.004\n");
  return 0;
}

}

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  std::size_t track = 0;
  int pwm = atlas::firmwareSafeConfig().speed.basePwm;
  std::vector<std::string> rest;
  for (std::size_t i = 0; i < args.size(); i++) {
    if (args[i] == "--pista" && i + 1 < args.size()) track = std::strtoul(args[++i].c_str(), nullptr, 10);
    else if (args[i] == "--pwm" && i + 1 < args.size()) pwm = std::atoi(args[++i].c_str());
    else rest.push_back(args[i]);
  }
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  if (track >= tracks.size()) {
    std::fprintf(stderr, "Pista fuera de rango (0-%zu)\n", tracks.size() - 1);
    return 1;
  }
  if (rest.size() >= 2 && rest[0] == "--sintetico") {
    Candidate c = {0.07, 0.04, 0.004};
    if (rest.size() >= 5) c = {std::atof(rest[2].c_str()), std::atof(rest[3].c_str()), std::atof(rest[4].c_str())};
    const std::vector<Frame> frames = simulate(tracks, track, pwm, 1, c, 999);
    writeLog(rest[1], frames);
    std::printf("Registro sintético: %zu tramas (motor %.3f giro %.3f latencia %.3f) en %s\n", frames.size(), c.motor,
                c.yaw, c.latency, rest[1].c_str());
    return 0;
  }
  if (rest.size() != 1) {
    std::fprintf(stderr,
                 "Uso: atlas_fit_robot vuelta.bin [--pista N] [--pwm P]\n"
                 "     atlas_fit_robot --sintetico salida.bin [motor giro latencia] [--pista N] [--pwm P]\n");
    return 1;
  }
  int rejected = 0;
  const std::vector<Frame> log = readLog(rest[0], &rejected);
  if (rejected) std::printf("Tramas con error de suma descartadas: %d\n", rejected);
  return fit(log, track, pwm, tracks);
}
