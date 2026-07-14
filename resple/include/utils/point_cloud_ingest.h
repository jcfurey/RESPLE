#pragma once

// ---------------------------------------------------------------------------
// point_cloud_ingest.h
//
// Thin ROS 2 + PCL wrapper around the dependency-light generic adapter
// (utils/point_cloud_adapter.h). It turns an arbitrary sensor_msgs/PointCloud2
// into the deskew-ready pcl::PointXYZINormal cloud the rest of RESPLE expects:
//
//     .x/.y/.z    <- the resolved x/y/z fields
//     .intensity  <- per-point time offset in milliseconds (>= 0)
//     .curvature  <- per-point reflectivity/intensity
//
// applying the same blind-radius and decimation (point_filter_num) filtering
// the hand-written per-sensor loaders did. Because all of the parsing logic
// lives in the (unit-tested) adapter, this file stays trivial and is exercised
// end-to-end by the opt-in integration tests (test/integration/).
//
// This header pulls in ROS 2 + PCL, so it is NOT part of the ROS-free unit-test
// build — only the node and the integration tests include it.
// ---------------------------------------------------------------------------

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "utils/point_cloud_adapter.h"

namespace resple {

// Build an AdapterConfig from the per-lidar YAML options (LidarConfig's
// time_field / time_unit / intensity_field strings). Empty overrides keep the
// built-in candidate lists; explicit names are tried FIRST but the defaults
// remain as fallback, so a slightly-wrong override degrades gracefully.
inline pc2::AdapterConfig makeAdapterConfig(const std::string& time_field,
                                            const std::string& time_unit,
                                            const std::string& intensity_field) {
  pc2::AdapterConfig cfg;
  if (!time_field.empty()) {
    cfg.names.time.insert(cfg.names.time.begin(), time_field);
  }
  if (!intensity_field.empty()) {
    cfg.names.intensity.insert(cfg.names.intensity.begin(), intensity_field);
  }
  if (time_unit == "s") {
    cfg.time_unit = pc2::TimeUnit::Seconds;
  } else if (time_unit == "ms") {
    cfg.time_unit = pc2::TimeUnit::Milliseconds;
  } else if (time_unit == "us") {
    cfg.time_unit = pc2::TimeUnit::Microseconds;
  } else if (time_unit == "ns") {
    cfg.time_unit = pc2::TimeUnit::Nanoseconds;
  } else {
    cfg.time_unit = pc2::TimeUnit::Auto;  // "auto" or anything unrecognized
  }
  return cfg;
}

// Build the adapter's field-descriptor list straight from a PointCloud2.
inline std::vector<pc2::FieldInfo> toFieldInfo(
    const sensor_msgs::msg::PointCloud2& msg) {
  std::vector<pc2::FieldInfo> fields;
  fields.reserve(msg.fields.size());
  for (const auto& f : msg.fields) {
    fields.push_back(pc2::FieldInfo{f.name, f.offset, f.datatype, f.count});
  }
  return fields;
}

// Convert a PointCloud2 into a deskew-ready pcl::PointXYZINormal cloud.
//
//   blind             - drop points within this radius (metres); <= 0 disables.
//   point_filter_num  - keep 1-in-N points (decimation); <= 1 keeps all.
//   cfg               - adapter config (field-name overrides, time unit).
//
// Returns false if the cloud has no usable x/y/z layout (a throttled warning is
// the caller's responsibility). `out_cloud` is cleared and filled.
inline bool ingestPointCloud2(const sensor_msgs::msg::PointCloud2& msg,
                              float blind, int point_filter_num,
                              const pc2::AdapterConfig& cfg,
                              pcl::PointCloud<pcl::PointXYZINormal>& out_cloud) {
  out_cloud.clear();
  const auto fields = toFieldInfo(msg);
  const pc2::ResolvedLayout layout = pc2::resolveLayout(fields, cfg);
  if (!layout.valid) {
    return false;
  }
  const uint32_t num_points = msg.width * msg.height;
  // convertCloud walks the blob at a fixed point_step, which assumes dense
  // packing. Organized clouds (height > 1) may pad each row to row_step >
  // width*point_step — repack them densely first, or every point after row 0
  // is read at the wrong offset (bug-hunt 2026-07-02 finding #26).
  const uint8_t* data_ptr = msg.data.data();
  size_t data_size = msg.data.size();
  std::vector<uint8_t> dense;
  if (msg.height > 1 &&
      msg.row_step != msg.width * msg.point_step) {
    // A row_step SMALLER than width*point_step (malformed) would make the
    // per-row memcpy of width*point_step bytes overread `msg.data`: the
    // row_step*height bound below does not cover it (it is < width*point_step*
    // height). Reject rather than read past the buffer.
    if (static_cast<size_t>(msg.row_step) <
            static_cast<size_t>(msg.width) * msg.point_step ||
        static_cast<size_t>(msg.row_step) * msg.height > msg.data.size()) {
      return false;
    }
    dense.resize(static_cast<size_t>(msg.width) * msg.height * msg.point_step);
    for (uint32_t r = 0; r < msg.height; ++r) {
      std::memcpy(dense.data() +
                      static_cast<size_t>(r) * msg.width * msg.point_step,
                  msg.data.data() + static_cast<size_t>(r) * msg.row_step,
                  static_cast<size_t>(msg.width) * msg.point_step);
    }
    data_ptr = dense.data();
    data_size = dense.size();
  }
  std::vector<pc2::OutPoint> pts;
  if (!pc2::convertCloud(data_ptr, data_size, msg.point_step,
                         num_points, msg.is_bigendian, layout, cfg, pts)) {
    return false;
  }

  const float blind_sq = blind > 0.f ? blind * blind : 0.f;
  const int stride = point_filter_num > 1 ? point_filter_num : 1;
  // Per-point RELATIVE time-offset ceiling (ms). Non-finite values pass both
  // gates below (NaN comparisons are false), and a finite-but-huge offset
  // (malformed field surviving the auto-scaler) overflows the int64 ns cast
  // downstream (CommonUtils::ms2ns in RESPLE, transformCloud in Mapping) —
  // UB, the hazards-80/82 threat model. 10 minutes is orders above any real
  // scan and orders below the cast limit. Gating HERE covers both nodes'
  // generic paths in one place.
  constexpr float kMaxTimeOffsetMs = 6.0e5f;
  out_cloud.points.reserve(pts.size() / static_cast<size_t>(stride) + 1);
  for (size_t i = 0; i < pts.size(); ++i) {
    if (stride > 1 && (i % static_cast<size_t>(stride)) != 0) {
      continue;
    }
    const pc2::OutPoint& s = pts[i];
    if (!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.z) ||
        !std::isfinite(s.time_ms) || s.time_ms > kMaxTimeOffsetMs) {
      continue;
    }
    const float r_sq = s.x * s.x + s.y * s.y + s.z * s.z;
    if (s.time_ms < 0.f || r_sq <= blind_sq) {
      continue;
    }
    pcl::PointXYZINormal pt;
    pt.x = s.x;
    pt.y = s.y;
    pt.z = s.z;
    pt.intensity = s.time_ms;  // deskew time offset (ms) — RESPLE convention
    pt.curvature = s.intensity;
    out_cloud.points.push_back(pt);
  }
  out_cloud.width = out_cloud.points.size();
  out_cloud.height = 1;
  out_cloud.is_dense = false;
  return true;
}

}  // namespace resple
