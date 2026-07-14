#pragma once

// ---------------------------------------------------------------------------
// filter_health.h
//
// Dependency-light (Eigen + std only) runtime health monitors for the
// estimator — HARDENING §3.3 (divergence detection) plus IMU input-quality
// monitoring aimed at poor IMUs (e.g. the Ouster built-in):
//
//   * NisDivergenceDetector — windowed NIS (normalized innovation squared)
//     consistency monitor.  A healthy, consistent filter has E[NIS] = dof;
//     a sustained NIS far above dof means the filter is over-confident /
//     diverging and the caller should react (reset covariance, raise a
//     diagnostic, fall back to LO mode...).
//
//   * ImuHealthMonitor — windowed statistics over raw IMU samples detecting
//     NaN/saturation/stuck-sensor faults, timestamp irregularities, noise
//     floors beyond configuration, and (when approximately stationary)
//     accel/gyro bias estimates.
//
// Both are intentionally free of ROS so they compile in the ROS-free unit
// suite (see test/test_filter_health.cpp).  The node wires their outputs into
// diagnostics and gating decisions.
// ---------------------------------------------------------------------------

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <deque>

namespace resple {
namespace health {

// ----------------------------- NIS divergence -------------------------------

enum class FilterState { OK, WARN, DIVERGED };

struct NisDetectorConfig {
  // Tumbling (non-overlapping) window length in updates: a verdict is produced
  // once per `window` samples over DISJOINT blocks (window_ is cleared after
  // each), and breach_limit/recover_limit count CONSECUTIVE such blocks — so
  // DIVERGED needs ~breach_limit*window sustained high-NIS samples. This is not
  // a per-sample sliding mean.
  int window = 32;
  // WARN when windowed mean NIS exceeds warn_ratio * dof.
  double warn_ratio = 2.0;
  // DIVERGED after `breach_limit` consecutive windows whose mean exceeds
  // diverged_ratio * dof.
  double diverged_ratio = 4.0;
  int breach_limit = 3;
  // Number of consecutive healthy windows required to recover back to OK.
  int recover_limit = 2;
};

// Windowed NIS consistency monitor.  feed() one NIS value (with its
// measurement dof) per IEKF update; non-finite NIS values (the nis() helper
// returns NaN when S is not SPD) count as immediate breaches since they mean
// the update itself was numerically unusable.
class NisDivergenceDetector {
 public:
  explicit NisDivergenceDetector(const NisDetectorConfig& cfg = {})
      : cfg_(cfg) {}

  FilterState feed(double nis_value, int dof) {
    if (dof <= 0) {
      return state_;
    }
    if (!std::isfinite(nis_value)) {
      // A non-finite innovation covariance is the strongest divergence signal
      // there is: skip the windowing and count a full breach directly.
      ++consecutive_breaches_;
      consecutive_healthy_ = 0;
      update_state();
      return state_;
    }
    window_.push_back(nis_value / static_cast<double>(dof));
    if (static_cast<int>(window_.size()) < cfg_.window) {
      return state_;  // not enough samples for a verdict yet
    }
    double mean = 0.0;
    for (double v : window_) {
      mean += v;
    }
    mean /= static_cast<double>(window_.size());
    last_window_mean_ = mean;
    window_.clear();

    if (mean > cfg_.diverged_ratio) {
      ++consecutive_breaches_;
      consecutive_healthy_ = 0;
    } else if (mean > cfg_.warn_ratio) {
      warned_ = true;
      consecutive_healthy_ = 0;
    } else {
      ++consecutive_healthy_;
      if (consecutive_healthy_ >= cfg_.recover_limit) {
        consecutive_breaches_ = 0;
        warned_ = false;
      }
    }
    update_state();
    return state_;
  }

  FilterState state() const { return state_; }
  // Mean of the most recently completed window, normalized by dof (so 1.0 is
  // the consistent-filter expectation). 0 until the first window completes.
  double lastWindowMean() const { return last_window_mean_; }
  std::uint64_t totalBreaches() const { return total_breaches_; }

  void reset() {
    window_.clear();
    consecutive_breaches_ = 0;
    consecutive_healthy_ = 0;
    warned_ = false;
    state_ = FilterState::OK;
    last_window_mean_ = 0.0;
  }

 private:
  void update_state() {
    if (consecutive_breaches_ >= cfg_.breach_limit) {
      if (state_ != FilterState::DIVERGED) {
        ++total_breaches_;
      }
      state_ = FilterState::DIVERGED;
    } else if (consecutive_breaches_ > 0 || warned_) {
      state_ = FilterState::WARN;
    } else {
      state_ = FilterState::OK;
    }
  }

  NisDetectorConfig cfg_;
  std::deque<double> window_;
  int consecutive_breaches_ = 0;
  int consecutive_healthy_ = 0;
  bool warned_ = false;
  FilterState state_ = FilterState::OK;
  double last_window_mean_ = 0.0;
  std::uint64_t total_breaches_ = 0;
};

// ------------------------------- IMU health ---------------------------------

// Bitmask of detected IMU faults.
enum ImuFault : std::uint32_t {
  IMU_OK = 0,
  IMU_NONFINITE = 1u << 0,     // NaN/Inf in a sample
  IMU_ACCEL_SATURATED = 1u << 1,  // |accel| beyond plausible full scale
  IMU_GYRO_SATURATED = 1u << 2,
  IMU_STUCK = 1u << 3,         // identical consecutive samples
  IMU_TIME_JUMP = 1u << 4,     // dt gap or non-monotonic timestamps
  IMU_NOISY = 1u << 5,         // stationary noise floor above configuration
  IMU_BIAS_LARGE = 1u << 6,    // stationary bias estimate above configuration
};

struct ImuHealthConfig {
  // Window length (samples) for the running statistics.  At the Ouster
  // built-in's 100 Hz this is ~1 s of data.
  int window = 100;
  // Saturation thresholds (m/s^2, rad/s).  Defaults fit low-cost MEMS parts
  // (e.g. ±16 g, ±2000 deg/s full scale) with margin.
  double accel_saturation = 150.0;
  double gyro_saturation = 30.0;
  // Consecutive identical samples before declaring the sensor stuck.  Real
  // MEMS noise makes exact repeats vanishingly unlikely.
  int stuck_limit = 20;
  // dt is "irregular" when above gap_factor * nominal_dt, or <= 0.
  double nominal_dt = 0.01;  // 100 Hz
  double gap_factor = 5.0;
  // Stationarity gate: the window counts as stationary when the gyro RMS is
  // below this (rad/s).  Bias/noise checks only run on stationary windows.
  double stationary_gyro_rms = 0.05;
  // Stationary-window accel noise floor (std dev, m/s^2) above which the IMU
  // is flagged noisy; generous default for poor built-in IMUs.
  double accel_noise_std_max = 0.8;
  // Stationary accel-bias magnitude bound (m/s^2): | mean(accel) - g | with
  // gravity removed along the measured direction.
  double accel_bias_max = 1.5;
  double gravity_norm = 9.80665;
};

struct ImuHealthReport {
  std::uint32_t faults = IMU_OK;     // OR of ImuFault bits (current window)
  bool stationary = false;           // last completed window was stationary
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();   // stationary mean
  double accel_bias_norm = 0.0;      // | |mean accel| - g | when stationary
  double accel_noise_std = 0.0;      // mean per-axis std dev (stationary)
  std::uint64_t samples = 0;         // total samples fed
  bool ok() const { return faults == IMU_OK; }
};

// Feed every raw IMU sample; the report aggregates instantaneous faults
// (non-finite, saturation, stuck, time jumps) immediately and refreshes the
// windowed statistics (noise, bias, stationarity) every `window` samples.
class ImuHealthMonitor {
 public:
  explicit ImuHealthMonitor(const ImuHealthConfig& cfg = {}) : cfg_(cfg) {}

  const ImuHealthReport& feed(std::int64_t t_ns, const Eigen::Vector3d& gyro,
                              const Eigen::Vector3d& accel) {
    ++report_.samples;
    std::uint32_t instant = IMU_OK;

    if (!gyro.allFinite() || !accel.allFinite()) {
      instant |= IMU_NONFINITE;
      // Don't fold garbage into the statistics.
      latch(instant);
      return report_;
    }
    if (accel.norm() > cfg_.accel_saturation) {
      instant |= IMU_ACCEL_SATURATED;
    }
    if (gyro.norm() > cfg_.gyro_saturation) {
      instant |= IMU_GYRO_SATURATED;
    }

    if (have_prev_) {
      const double dt = static_cast<double>(t_ns - prev_t_ns_) * 1e-9;
      if (dt <= 0.0 || dt > cfg_.gap_factor * cfg_.nominal_dt) {
        instant |= IMU_TIME_JUMP;
      }
      if (gyro == prev_gyro_ && accel == prev_accel_) {
        if (++stuck_count_ >= cfg_.stuck_limit) {
          instant |= IMU_STUCK;
        }
      } else {
        stuck_count_ = 0;
      }
    }
    prev_t_ns_ = t_ns;
    prev_gyro_ = gyro;
    prev_accel_ = accel;
    have_prev_ = true;

    if (instant != IMU_OK) {
      // A faulted sample would poison the windowed statistics — e.g. a single
      // saturated reading skews the stationary-bias mean for the whole window
      // and false-flags IMU_BIAS_LARGE. Latch the fault (visible until a full
      // clean window completes) and keep the sample out of the stats.
      latch(instant);
      return report_;
    }

    // Welford running statistics for the window.
    ++n_;
    const Eigen::Vector3d d_g = gyro - mean_gyro_;
    mean_gyro_ += d_g / static_cast<double>(n_);
    m2_gyro_ += d_g.cwiseProduct(gyro - mean_gyro_);
    const Eigen::Vector3d d_a = accel - mean_accel_;
    mean_accel_ += d_a / static_cast<double>(n_);
    m2_accel_ += d_a.cwiseProduct(accel - mean_accel_);

    if (n_ >= cfg_.window) {
      finishWindow(instant);
    } else {
      latch(instant);
    }
    return report_;
  }

  const ImuHealthReport& report() const { return report_; }

  void reset() {
    report_ = ImuHealthReport{};
    have_prev_ = false;
    stuck_count_ = 0;
    resetWindow();
  }

 private:
  void resetWindow() {
    n_ = 0;
    mean_gyro_.setZero();
    mean_accel_.setZero();
    m2_gyro_.setZero();
    m2_accel_.setZero();
  }

  void latch(std::uint32_t instant) {
    // Instantaneous faults stay visible until the next completed window so a
    // 10 ms glitch is not missed between two diagnostic polls.
    report_.faults |= instant;
  }

  void finishWindow(std::uint32_t instant) {
    const double inv_n1 = n_ > 1 ? 1.0 / static_cast<double>(n_ - 1) : 0.0;
    const Eigen::Vector3d var_gyro = m2_gyro_ * inv_n1;
    const Eigen::Vector3d var_accel = m2_accel_ * inv_n1;
    const double gyro_rms = std::sqrt(
        (mean_gyro_.squaredNorm() + var_gyro.sum()) / 3.0);

    std::uint32_t windowed = instant;
    const bool stationary = gyro_rms < cfg_.stationary_gyro_rms;
    double accel_noise_std =
        std::sqrt(var_accel.sum() / 3.0);
    double accel_bias_norm = std::fabs(mean_accel_.norm() - cfg_.gravity_norm);
    if (stationary) {
      if (accel_noise_std > cfg_.accel_noise_std_max) {
        windowed |= IMU_NOISY;
      }
      if (accel_bias_norm > cfg_.accel_bias_max) {
        windowed |= IMU_BIAS_LARGE;
      }
    }

    // Replace (not OR) the report at window boundaries: a fault that stopped
    // occurring clears once a full clean window completes.
    report_.faults = windowed;
    report_.stationary = stationary;
    if (stationary) {
      report_.gyro_bias = mean_gyro_;
      report_.accel_bias_norm = accel_bias_norm;
      report_.accel_noise_std = accel_noise_std;
    }
    resetWindow();
  }

  ImuHealthConfig cfg_;
  ImuHealthReport report_;

  bool have_prev_ = false;
  std::int64_t prev_t_ns_ = 0;
  Eigen::Vector3d prev_gyro_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_accel_ = Eigen::Vector3d::Zero();
  int stuck_count_ = 0;

  int n_ = 0;
  Eigen::Vector3d mean_gyro_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_accel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d m2_gyro_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d m2_accel_ = Eigen::Vector3d::Zero();
};

}  // namespace health
}  // namespace resple
