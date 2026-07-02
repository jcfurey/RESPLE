#pragma once

// ---------------------------------------------------------------------------
// point_cloud_adapter.h
//
// Generic, runtime PointCloud2 field introspection — the core that lets RESPLE
// ingest "virtually any" sensor_msgs/PointCloud2 instead of relying on a
// hand-written PCL point struct per sensor model.
//
// It is deliberately free of ROS 2 and PCL (it works on the raw byte buffer +
// a vector of field descriptors) so it can be compiled and unit-tested on its
// own (see test/test_point_cloud_adapter.cpp). The thin ROS wrapper that
// builds a FieldInfo list from a sensor_msgs::msg::PointCloud2 and emits
// pcl::PointXYZINormal lives in the node and simply calls convertCloud().
//
// What it solves that the per-sensor loaders did not:
//   * resolves x/y/z and the time / intensity / ring fields by NAME from a
//     configurable candidate list (so Ouster `t`, Velodyne `time`, Hesai
//     `timestamp`, Livox `offset_time`, ... all just work),
//   * reads each field regardless of its declared datatype and endianness,
//   * normalizes the per-point time into a non-negative millisecond offset
//     from scan start, transparently handling both relative time fields
//     (offset already ~0-based) and absolute-epoch time fields (subtract the
//     per-cloud minimum). This matches RESPLE's `.intensity = ms-offset`
//     deskew convention.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace resple {
namespace pc2 {

// Mirrors sensor_msgs::msg::PointField::<TYPE> datatype constants so the ROS
// wrapper can pass msg field datatypes straight through.
enum Datatype : uint8_t {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8,
};

// Mirrors sensor_msgs::msg::PointField.
struct FieldInfo {
  std::string name;
  uint32_t offset = 0;
  uint8_t datatype = 0;
  uint32_t count = 1;
};

inline size_t datatypeSize(uint8_t dt) {
  switch (dt) {
    case INT8:
    case UINT8:
      return 1;
    case INT16:
    case UINT16:
      return 2;
    case INT32:
    case UINT32:
    case FLOAT32:
      return 4;
    case FLOAT64:
      return 8;
    default:
      return 0;
  }
}

// Read `n` bytes from `src` into a trivially-copyable value, byte-swapping when
// the message endianness differs from the host. Templated so the caller never
// reinterpret_casts misaligned data (the byte copy is alignment-safe).
template <typename T>
inline T readScalar(const uint8_t* src, bool is_bigendian) {
  T value;
  std::memcpy(&value, src, sizeof(T));
  // Host is assumed little-endian (x86_64 / aarch64 in all supported targets).
  if (is_bigendian) {
    uint8_t* b = reinterpret_cast<uint8_t*>(&value);
    std::reverse(b, b + sizeof(T));
  }
  return value;
}

// Read one field of one point as a double, regardless of declared datatype.
// `point_ptr` is the start of the point's row (data + i * point_step).
inline double readFieldAsDouble(const uint8_t* point_ptr, const FieldInfo& f,
                                bool is_bigendian) {
  const uint8_t* p = point_ptr + f.offset;
  switch (f.datatype) {
    case INT8:
      return static_cast<double>(static_cast<int8_t>(*p));
    case UINT8:
      return static_cast<double>(*p);
    case INT16:
      return static_cast<double>(readScalar<int16_t>(p, is_bigendian));
    case UINT16:
      return static_cast<double>(readScalar<uint16_t>(p, is_bigendian));
    case INT32:
      return static_cast<double>(readScalar<int32_t>(p, is_bigendian));
    case UINT32:
      return static_cast<double>(readScalar<uint32_t>(p, is_bigendian));
    case FLOAT32:
      return static_cast<double>(readScalar<float>(p, is_bigendian));
    case FLOAT64:
      return readScalar<double>(p, is_bigendian);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

enum class TimeUnit { Auto, Seconds, Milliseconds, Microseconds, Nanoseconds };

// Candidate field names, tried in order. Defaults cover the common LiDAR
// conventions; a node can extend/override them from parameters.
struct FieldNames {
  std::vector<std::string> x{"x"};
  std::vector<std::string> y{"y"};
  std::vector<std::string> z{"z"};
  std::vector<std::string> time{"t",        "time",        "timestamp",
                                "time_stamp", "offset_time", "time_offset",
                                "ts",        "t_sec"};
  std::vector<std::string> intensity{"intensity", "reflectivity", "i"};
  std::vector<std::string> ring{"ring", "line", "channel"};
};

struct AdapterConfig {
  FieldNames names;
  // Auto: integer time fields are treated as nanoseconds, floating-point ones
  // as seconds (the near-universal conventions). Override per sensor if a
  // driver is unusual (e.g. microseconds).
  TimeUnit time_unit = TimeUnit::Auto;
  // Subtract the per-cloud minimum time so an absolute-epoch field (Hesai
  // double seconds, Livox int64 ns) and a relative field (Ouster ns offset)
  // both yield offsets measured from scan start. Disable only if a driver
  // already guarantees 0-based offsets and you want to skip the min pass.
  bool normalize_time_to_min = true;
};

// Resolved field handles for a fixed cloud layout. Resolve once per layout
// (the PointCloud2 field set is constant for a given sensor) and reuse.
struct ResolvedLayout {
  bool valid = false;
  std::string error;
  FieldInfo x, y, z;
  bool has_time = false, has_intensity = false;
  FieldInfo time, intensity;
};

namespace detail {
inline const FieldInfo* findField(const std::vector<FieldInfo>& fields,
                                   const std::vector<std::string>& candidates) {
  for (const auto& cand : candidates) {
    for (const auto& f : fields) {
      if (f.name == cand) {
        return &f;
      }
    }
  }
  return nullptr;
}
}  // namespace detail

// Resolve x/y/z (required) and time/intensity (optional) from a field list.
inline ResolvedLayout resolveLayout(const std::vector<FieldInfo>& fields,
                                    const AdapterConfig& cfg = {}) {
  ResolvedLayout out;
  const FieldInfo* fx = detail::findField(fields, cfg.names.x);
  const FieldInfo* fy = detail::findField(fields, cfg.names.y);
  const FieldInfo* fz = detail::findField(fields, cfg.names.z);
  if (!fx || !fy || !fz) {
    out.error = "PointCloud2 is missing one of the x/y/z fields";
    return out;
  }
  out.x = *fx;
  out.y = *fy;
  out.z = *fz;
  if (const FieldInfo* ft = detail::findField(fields, cfg.names.time)) {
    out.time = *ft;
    out.has_time = true;
  }
  if (const FieldInfo* fi = detail::findField(fields, cfg.names.intensity)) {
    out.intensity = *fi;
    out.has_intensity = true;
  }
  out.valid = true;
  return out;
}

inline double timeUnitToMs(TimeUnit unit, uint8_t datatype) {
  switch (unit) {
    case TimeUnit::Seconds:
      return 1.0e3;
    case TimeUnit::Milliseconds:
      return 1.0;
    case TimeUnit::Microseconds:
      return 1.0e-3;
    case TimeUnit::Nanoseconds:
      return 1.0e-6;
    case TimeUnit::Auto:
    default:
      // Float fields are conventionally seconds; integer fields nanoseconds.
      return (datatype == FLOAT32 || datatype == FLOAT64) ? 1.0e3 : 1.0e-6;
  }
}

// One converted point: xyz, the deskew time offset in milliseconds (>= 0 after
// normalization) and intensity/reflectivity. Mirrors what the node writes into
// pcl::PointXYZINormal (.intensity <- time_ms, .curvature <- intensity).
struct OutPoint {
  float x = 0.f, y = 0.f, z = 0.f;
  float time_ms = 0.f;
  float intensity = 0.f;
};

// Convert a full cloud. `data` is the PointCloud2 byte blob; `num_points`
// should be width*height. Returns false if the layout is invalid or the buffer
// is too small for the declared geometry. `out` is cleared and filled.
inline bool convertCloud(const uint8_t* data, size_t data_size,
                         uint32_t point_step, uint32_t num_points,
                         bool is_bigendian, const ResolvedLayout& layout,
                         const AdapterConfig& cfg, std::vector<OutPoint>& out) {
  out.clear();
  if (!layout.valid) {
    return false;
  }
  if (point_step == 0 ||
      static_cast<size_t>(num_points) * point_step > data_size) {
    return false;
  }
  out.resize(num_points);

  double t_scale =
      layout.has_time ? timeUnitToMs(cfg.time_unit, layout.time.datatype) : 0.0;
  // Auto refinement by magnitude (bug-hunt 2026-07-02 finding #23): the
  // datatype convention alone (float=seconds, int=nanoseconds) misreads
  // float64 NANOSECOND stamps (e.g. livox_ros_driver2's 'timestamp') as
  // seconds — a 1e9x deskew-time error. Absolute-epoch stamps are cleanly
  // separable by magnitude (2026 epoch: ~1.7e18 ns / ~1.7e15 us / ~1.7e12 ms /
  // ~1.7e9 s), far above any RELATIVE per-point offset (< ~5e8 even for a
  // 0.5 s scan in ns). Relative offsets keep the datatype convention.
  if (layout.has_time && cfg.time_unit == TimeUnit::Auto && num_points > 0) {
    double v_max = 0.0;
    const uint32_t n_probe = num_points < 64u ? num_points : 64u;
    for (uint32_t i = 0; i < n_probe; ++i) {
      const uint8_t* pp = data + static_cast<size_t>(i) * point_step;
      v_max = std::max(v_max,
                       std::abs(readFieldAsDouble(pp, layout.time, is_bigendian)));
    }
    if (v_max > 1e17) {
      t_scale = 1.0e-6;  // absolute epoch, nanoseconds
    } else if (v_max > 1e14) {
      t_scale = 1.0e-3;  // absolute epoch, microseconds
    } else if (v_max > 1e11) {
      t_scale = 1.0;     // absolute epoch, milliseconds
    } else if (v_max > 5e8) {
      t_scale = 1.0e3;   // absolute epoch, seconds
    }
  }

  // Keep per-point time in DOUBLE milliseconds until after normalization: an
  // absolute-epoch field (e.g. ~1.7e12 ms) overflows float32's ~7 significant
  // digits, so casting before subtracting the scan-start min would erase the
  // sub-millisecond deskew offsets we are trying to recover.
  std::vector<double> t_ms_raw;
  if (layout.has_time) {
    t_ms_raw.resize(num_points);
  }
  double min_time_ms = std::numeric_limits<double>::infinity();
  for (uint32_t i = 0; i < num_points; ++i) {
    const uint8_t* pp = data + static_cast<size_t>(i) * point_step;
    OutPoint& o = out[i];
    o.x = static_cast<float>(readFieldAsDouble(pp, layout.x, is_bigendian));
    o.y = static_cast<float>(readFieldAsDouble(pp, layout.y, is_bigendian));
    o.z = static_cast<float>(readFieldAsDouble(pp, layout.z, is_bigendian));
    if (layout.has_intensity) {
      o.intensity = static_cast<float>(
          readFieldAsDouble(pp, layout.intensity, is_bigendian));
    }
    if (layout.has_time) {
      const double t_ms =
          readFieldAsDouble(pp, layout.time, is_bigendian) * t_scale;
      t_ms_raw[i] = t_ms;
      if (t_ms < min_time_ms) {
        min_time_ms = t_ms;
      }
    }
  }

  // Rebase onto scan start so the offset is non-negative whether the source
  // field was relative (min ~ 0) or an absolute epoch (min ~ scan start).
  if (layout.has_time) {
    const double base =
        (cfg.normalize_time_to_min && std::isfinite(min_time_ms)) ? min_time_ms
                                                                  : 0.0;
    for (uint32_t i = 0; i < num_points; ++i) {
      out[i].time_ms = static_cast<float>(t_ms_raw[i] - base);
    }
  }
  return true;
}

}  // namespace pc2
}  // namespace resple
