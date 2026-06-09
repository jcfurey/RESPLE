// Unit tests for resple/include/utils/filter_health.h
//
// ROS-free (Eigen + GoogleTest): exercises the windowed NIS divergence
// detector and the IMU health monitor with synthetic, deterministic streams.

#include "utils/filter_health.h"

#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <random>

using namespace resple::health;

namespace {

constexpr int kDof = 6;

NisDetectorConfig fastNisCfg() {
  NisDetectorConfig cfg;
  cfg.window = 8;
  cfg.warn_ratio = 2.0;
  cfg.diverged_ratio = 4.0;
  cfg.breach_limit = 3;
  cfg.recover_limit = 2;
  return cfg;
}

}  // namespace

// --------------------------- NisDivergenceDetector --------------------------

TEST(NisDetector, ConsistentFilterStaysOk) {
  NisDivergenceDetector det(fastNisCfg());
  // NIS hovering around dof (the consistent expectation).
  for (int i = 0; i < 200; ++i) {
    det.feed(kDof * (0.8 + 0.4 * ((i % 5) / 4.0)), kDof);
  }
  EXPECT_EQ(det.state(), FilterState::OK);
  EXPECT_NEAR(det.lastWindowMean(), 1.0, 0.3);
}

TEST(NisDetector, SustainedHighNisDiverges) {
  auto cfg = fastNisCfg();
  NisDivergenceDetector det(cfg);
  // NIS at 10x dof: every window breaches; DIVERGED after
  // breach_limit windows of data.
  const int updates_to_diverge = cfg.window * cfg.breach_limit;
  FilterState st = FilterState::OK;
  for (int i = 0; i < updates_to_diverge; ++i) {
    st = det.feed(10.0 * kDof, kDof);
  }
  EXPECT_EQ(st, FilterState::DIVERGED);
  EXPECT_GE(det.totalBreaches(), 1u);
}

TEST(NisDetector, ModeratelyHighNisWarns) {
  auto cfg = fastNisCfg();
  NisDivergenceDetector det(cfg);
  // 3x dof: above warn_ratio (2) but below diverged_ratio (4).
  for (int i = 0; i < cfg.window * 4; ++i) {
    det.feed(3.0 * kDof, kDof);
  }
  EXPECT_EQ(det.state(), FilterState::WARN);
}

TEST(NisDetector, NonFiniteNisCountsAsBreach) {
  auto cfg = fastNisCfg();
  NisDivergenceDetector det(cfg);
  for (int i = 0; i < cfg.breach_limit; ++i) {
    det.feed(std::numeric_limits<double>::quiet_NaN(), kDof);
  }
  EXPECT_EQ(det.state(), FilterState::DIVERGED);
}

TEST(NisDetector, RecoversAfterHealthyWindows) {
  auto cfg = fastNisCfg();
  NisDivergenceDetector det(cfg);
  for (int i = 0; i < cfg.window * cfg.breach_limit; ++i) {
    det.feed(10.0 * kDof, kDof);
  }
  ASSERT_EQ(det.state(), FilterState::DIVERGED);
  // recover_limit consecutive healthy windows clear the breach counter.
  for (int i = 0; i < cfg.window * cfg.recover_limit; ++i) {
    det.feed(1.0 * kDof, kDof);
  }
  EXPECT_EQ(det.state(), FilterState::OK);
}

// ------------------------------ ImuHealthMonitor ----------------------------

namespace {

ImuHealthConfig fastImuCfg() {
  ImuHealthConfig cfg;
  cfg.window = 50;
  cfg.nominal_dt = 0.01;
  return cfg;
}

// Deterministic Gaussian-ish noise.
struct Noise {
  std::mt19937 rng{42};
  std::normal_distribution<double> dist{0.0, 1.0};
  Eigen::Vector3d vec(double std_dev) {
    return std_dev * Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
  }
};

}  // namespace

TEST(ImuHealth, CleanStationaryImuIsOkWithBiasEstimate) {
  auto cfg = fastImuCfg();
  ImuHealthMonitor mon(cfg);
  Noise noise;
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.005);
  const Eigen::Vector3d g(0, 0, cfg.gravity_norm);
  std::int64_t t = 0;
  for (int i = 0; i < cfg.window * 3; ++i) {
    t += 10'000'000;  // 10 ms
    mon.feed(t, gyro_bias + noise.vec(0.002), g + noise.vec(0.05));
  }
  const auto& rep = mon.report();
  EXPECT_TRUE(rep.ok()) << "faults=" << rep.faults;
  EXPECT_TRUE(rep.stationary);
  EXPECT_NEAR((rep.gyro_bias - gyro_bias).norm(), 0.0, 0.005);
  EXPECT_LT(rep.accel_bias_norm, 0.2);
}

TEST(ImuHealth, NonFiniteSampleFlags) {
  ImuHealthMonitor mon(fastImuCfg());
  Eigen::Vector3d bad(std::nan(""), 0, 0);
  mon.feed(10'000'000, bad, Eigen::Vector3d(0, 0, 9.81));
  EXPECT_TRUE(mon.report().faults & IMU_NONFINITE);
}

TEST(ImuHealth, SaturationFlags) {
  ImuHealthMonitor mon(fastImuCfg());
  mon.feed(10'000'000, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 500.0));
  EXPECT_TRUE(mon.report().faults & IMU_ACCEL_SATURATED);
  mon.reset();
  mon.feed(10'000'000, Eigen::Vector3d(100, 0, 0), Eigen::Vector3d(0, 0, 9.81));
  EXPECT_TRUE(mon.report().faults & IMU_GYRO_SATURATED);
}

TEST(ImuHealth, StuckSensorFlags) {
  auto cfg = fastImuCfg();
  ImuHealthMonitor mon(cfg);
  const Eigen::Vector3d gyro(0.001, 0.002, 0.003);
  const Eigen::Vector3d accel(0, 0, 9.81);
  std::int64_t t = 0;
  for (int i = 0; i <= cfg.stuck_limit + 1; ++i) {
    t += 10'000'000;
    mon.feed(t, gyro, accel);  // exactly identical every sample
  }
  EXPECT_TRUE(mon.report().faults & IMU_STUCK);
}

TEST(ImuHealth, TimeJumpFlags) {
  ImuHealthMonitor mon(fastImuCfg());
  Noise noise;
  const Eigen::Vector3d g(0, 0, 9.81);
  mon.feed(10'000'000, noise.vec(0.01), g + noise.vec(0.05));
  // 1 s gap >> gap_factor * nominal_dt (50 ms)
  mon.feed(1'010'000'000, noise.vec(0.01), g + noise.vec(0.05));
  EXPECT_TRUE(mon.report().faults & IMU_TIME_JUMP);
  mon.reset();
  // Non-monotonic timestamp also flags.
  mon.feed(2'000'000'000, noise.vec(0.01), g + noise.vec(0.05));
  mon.feed(1'999'000'000, noise.vec(0.01), g + noise.vec(0.05));
  EXPECT_TRUE(mon.report().faults & IMU_TIME_JUMP);
}

TEST(ImuHealth, NoisyStationaryImuFlagsAfterWindow) {
  auto cfg = fastImuCfg();
  cfg.accel_noise_std_max = 0.1;  // tight bound so synthetic noise trips it
  ImuHealthMonitor mon(cfg);
  Noise noise;
  const Eigen::Vector3d g(0, 0, cfg.gravity_norm);
  std::int64_t t = 0;
  for (int i = 0; i < cfg.window; ++i) {
    t += 10'000'000;
    mon.feed(t, noise.vec(0.002), g + noise.vec(0.5));  // very noisy accel
  }
  EXPECT_TRUE(mon.report().faults & IMU_NOISY);
  EXPECT_TRUE(mon.report().stationary);
}

TEST(ImuHealth, FaultClearsAfterCleanWindow) {
  auto cfg = fastImuCfg();
  ImuHealthMonitor mon(cfg);
  Noise noise;
  const Eigen::Vector3d g(0, 0, cfg.gravity_norm);
  // One saturated sample latches the fault...
  mon.feed(10'000'000, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 500.0));
  ASSERT_FALSE(mon.report().ok());
  // ...then a full clean window replaces (clears) it.
  std::int64_t t = 10'000'000;
  for (int i = 0; i < cfg.window + 1; ++i) {
    t += 10'000'000;
    mon.feed(t, noise.vec(0.002), g + noise.vec(0.05));
  }
  EXPECT_TRUE(mon.report().ok()) << "faults=" << mon.report().faults;
}
