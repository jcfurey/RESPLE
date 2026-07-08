#pragma once

// ---------------------------------------------------------------------------
// sim_time_wait.h
//
// Dependency-light (std-only) ROS-time wait tracker (2026-07-08 clock-domain
// audit). Modeled on utils/overload_control.h: pure struct, no ROS,
// unit-tested in resple/test/test_sim_time_wait.cpp.
//
// Problem: "wait up to N seconds for X" timeouts (TF availability, IMU
// arrival, external-owner absence) used std::chrono::steady_clock — WALL
// time. On bag replay with --clock + use_sim_time at throttled rates, wall
// timeouts expire early relative to the data: at --rate 0.25 a 10 s TF wait
// covers only 2.5 s of bag time, so the node falls back (YAML extrinsics,
// identity attitude, absence WARN) before the bag ever publishes the data it
// is waiting for.
//
// RosTimeWait measures elapsed time in the NODE-CLOCK domain (sim time when
// use_sim_time is set, system time otherwise), fed by the caller as int64
// nanoseconds so this core stays ROS-free. It handles the two sim-time traps:
//
//   * clock-not-started: with use_sim_time, now() reads 0 until the first
//     /clock message. elapsedSec() returns 0 and does NOT start the window,
//     so a node activated before `ros2 bag play` begins cannot expire — and
//     when the clock then starts at bag time T0 (~1.7e18 ns), the window
//     starts AT T0 instead of measuring a bogus T0-sized jump.
//   * backwards jump (bag loop restart, seeked replay, NTP step live): the
//     wait restarts rather than wedging on a start stamp in the future.
// ---------------------------------------------------------------------------

#include <cstdint>

namespace resple {
namespace timeutil {

struct RosTimeWait {
  // 0 = not started (either reset() or the ROS clock hasn't started yet).
  int64_t start_ns = 0;

  // Elapsed seconds of ROS time since the first VALID clock sample after
  // (re)start. Call with the current node-clock reading (now().nanoseconds()).
  double elapsedSec(int64_t now_ns) {
    if (now_ns <= 0) {
      return 0.0;  // sim clock not publishing yet — don't start the window
    }
    if (start_ns == 0 || now_ns < start_ns) {
      start_ns = now_ns;  // first sample, or clock jumped backwards: restart
    }
    return static_cast<double>(now_ns - start_ns) / 1e9;
  }

  bool started() const { return start_ns != 0; }

  void reset() { start_ns = 0; }
};

}  // namespace timeutil
}  // namespace resple
