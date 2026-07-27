// Unit tests for resple/include/utils/point_cloud_adapter.h
//
// ROS-free: builds raw PointCloud2-style byte buffers in-process and checks the
// field introspection, multi-datatype/endianness reads, and time normalization
// that let RESPLE ingest arbitrary sensor_msgs/PointCloud2 layouts.

#include "utils/point_cloud_adapter.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

using namespace resple::pc2;

namespace {

// Helper to append a value's raw little-endian bytes into a buffer.
template <typename T>
void put(std::vector<uint8_t>& buf, size_t offset, T value) {
  if (buf.size() < offset + sizeof(T)) buf.resize(offset + sizeof(T));
  std::memcpy(buf.data() + offset, &value, sizeof(T));
}

}  // namespace

// ----- Ouster-like layout: x,y,z float32, t uint32 (ns), intensity float -----

TEST(PointCloudAdapter, OusterLikeRelativeNanosecondTime) {
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1},   {"y", 4, FLOAT32, 1},
      {"z", 8, FLOAT32, 1},   {"intensity", 12, FLOAT32, 1},
      {"t", 16, UINT32, 1},
  };
  const uint32_t point_step = 20;
  const uint32_t n = 3;
  std::vector<uint8_t> data(point_step * n, 0);
  // Three points with t = 0, 100000ns(=0.1ms), 1000000ns(=1ms).
  const uint32_t ts[3] = {0u, 100000u, 1000000u};
  const float xs[3] = {1.f, 2.f, 3.f};
  for (uint32_t i = 0; i < n; ++i) {
    size_t base = i * point_step;
    put<float>(data, base + 0, xs[i]);
    put<float>(data, base + 4, xs[i] + 0.5f);
    put<float>(data, base + 8, xs[i] + 1.0f);
    put<float>(data, base + 12, 10.f * i);
    put<uint32_t>(data, base + 16, ts[i]);
  }

  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid) << layout.error;
  ASSERT_TRUE(layout.has_time);
  ASSERT_TRUE(layout.has_intensity);

  std::vector<OutPoint> out;
  ASSERT_TRUE(convertCloud(data.data(), data.size(), point_step, n,
                           /*is_bigendian=*/false, layout, {}, out));
  ASSERT_EQ(out.size(), n);
  EXPECT_FLOAT_EQ(out[0].x, 1.f);
  EXPECT_FLOAT_EQ(out[2].z, 4.f);
  EXPECT_FLOAT_EQ(out[1].intensity, 10.f);
  // Auto unit: integer -> nanoseconds -> ms. Normalized to min (0).
  EXPECT_NEAR(out[0].time_ms, 0.0, 1e-6);
  EXPECT_NEAR(out[1].time_ms, 0.1, 1e-4);
  EXPECT_NEAR(out[2].time_ms, 1.0, 1e-4);
}

// ----- Hesai-like layout: absolute float64 seconds timestamp ----------------

TEST(PointCloudAdapter, HesaiLikeAbsoluteSecondsTimeNormalized) {
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1},
      {"y", 4, FLOAT32, 1},
      {"z", 8, FLOAT32, 1},
      {"timestamp", 16, FLOAT64, 1},
  };
  const uint32_t point_step = 24;
  const uint32_t n = 3;
  std::vector<uint8_t> data(point_step * n, 0);
  // Absolute epoch seconds; offsets are 0, 0.5ms, 2ms from the first.
  const double base_s = 1748505600.000;  // some large absolute time
  const double secs[3] = {base_s, base_s + 0.0005, base_s + 0.002};
  for (uint32_t i = 0; i < n; ++i) {
    size_t base = i * point_step;
    put<float>(data, base + 0, static_cast<float>(i));
    put<double>(data, base + 16, secs[i]);
  }

  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid) << layout.error;
  ASSERT_TRUE(layout.has_time);
  EXPECT_FALSE(layout.has_intensity);

  std::vector<OutPoint> out;
  ASSERT_TRUE(convertCloud(data.data(), data.size(), point_step, n, false,
                           layout, {}, out));
  // Float -> seconds -> ms, rebased to the first point.
  EXPECT_NEAR(out[0].time_ms, 0.0, 1e-3);
  EXPECT_NEAR(out[1].time_ms, 0.5, 1e-3);
  EXPECT_NEAR(out[2].time_ms, 2.0, 1e-3);
}

// ----- Field name resolution + alternates -----------------------------------

TEST(PointCloudAdapter, ResolvesAlternateFieldNames) {
  // Velodyne-ish: "time" (float seconds) + "ring".
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1},    {"y", 4, FLOAT32, 1},
      {"z", 8, FLOAT32, 1},    {"intensity", 12, FLOAT32, 1},
      {"ring", 16, UINT16, 1}, {"time", 18, FLOAT32, 1},
  };
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid);
  EXPECT_EQ(layout.time.name, "time");
  EXPECT_EQ(layout.time.datatype, FLOAT32);
}

TEST(PointCloudAdapter, FailsWhenXyzMissing) {
  std::vector<FieldInfo> fields = {{"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}};
  auto layout = resolveLayout(fields);
  EXPECT_FALSE(layout.valid);
  EXPECT_FALSE(layout.error.empty());
}

TEST(PointCloudAdapter, WorksWithoutTimeField) {
  // A bare XYZ cloud (e.g. a poor sensor / depth camera) must still convert;
  // time_ms is simply 0 everywhere (no deskew information available).
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}, {"z", 8, FLOAT32, 1}};
  const uint32_t point_step = 12, n = 2;
  std::vector<uint8_t> data(point_step * n, 0);
  put<float>(data, 0, 1.f);
  put<float>(data, point_step + 0, 2.f);
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid);
  EXPECT_FALSE(layout.has_time);
  std::vector<OutPoint> out;
  ASSERT_TRUE(convertCloud(data.data(), data.size(), point_step, n, false,
                           layout, {}, out));
  EXPECT_FLOAT_EQ(out[0].time_ms, 0.f);
  EXPECT_FLOAT_EQ(out[1].time_ms, 0.f);
}

// ----- Datatype + endianness reads ------------------------------------------

TEST(PointCloudAdapter, ReadsDatatypesAndEndianness) {
  // int16 value 0x0102 = 258; verify both endiannesses decode correctly.
  std::vector<uint8_t> le = {0x02, 0x01};
  FieldInfo f{"v", 0, INT16, 1};
  EXPECT_EQ(readFieldAsDouble(le.data(), f, /*is_bigendian=*/false), 258.0);
  std::vector<uint8_t> be = {0x01, 0x02};
  EXPECT_EQ(readFieldAsDouble(be.data(), f, /*is_bigendian=*/true), 258.0);

  EXPECT_EQ(datatypeSize(FLOAT64), 8u);
  EXPECT_EQ(datatypeSize(UINT8), 1u);
  EXPECT_EQ(datatypeSize(0 /*invalid*/), 0u);
}

TEST(PointCloudAdapter, RejectsTruncatedBuffer) {
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}, {"z", 8, FLOAT32, 1}};
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid);
  std::vector<uint8_t> data(12, 0);  // only room for 1 point
  std::vector<OutPoint> out;
  // Claim 5 points but supply room for 1 -> must fail, not read OOB.
  EXPECT_FALSE(convertCloud(data.data(), data.size(), 12, 5, false, layout, {},
                            out));
}

TEST(PointCloudAdapter, ExplicitTimeUnitOverride) {
  // Microsecond integer time field (some drivers): force the unit.
  std::vector<FieldInfo> fields = {{"x", 0, FLOAT32, 1},
                                   {"y", 4, FLOAT32, 1},
                                   {"z", 8, FLOAT32, 1},
                                   {"t", 12, UINT32, 1}};
  const uint32_t point_step = 16, n = 2;
  std::vector<uint8_t> data(point_step * n, 0);
  put<uint32_t>(data, 12, 0u);
  put<uint32_t>(data, point_step + 12, 2000u);  // 2000 us = 2 ms
  auto layout = resolveLayout(fields);
  AdapterConfig cfg;
  cfg.time_unit = TimeUnit::Microseconds;
  std::vector<OutPoint> out;
  ASSERT_TRUE(convertCloud(data.data(), data.size(), point_step, n, false,
                           layout, cfg, out));
  EXPECT_NEAR(out[1].time_ms, 2.0, 1e-4);
}

// ----- Auto time-unit magnitude sniffing (2026-07-02 finding #23) ----------
// The datatype convention alone (float=seconds, int=nanoseconds) misreads
// absolute-epoch stamps in the "wrong" datatype; Auto now sniffs magnitude.

TEST(PointCloudAdapter, AutoSniffsFloat64AbsoluteNanoseconds) {
  // livox_ros_driver2-style: 'timestamp' float64 holding EPOCH NANOSECONDS.
  // Under the old float=seconds convention the relative offsets came out
  // 1e9x too large; the magnitude sniff must classify this as ns.
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}, {"z", 8, FLOAT32, 1},
      {"timestamp", 12, FLOAT64, 1},
  };
  const uint32_t point_step = 20;
  const uint32_t n = 3;
  std::vector<uint8_t> data(point_step * n, 0);
  const double t0 = 1.7e18;  // ~2023 epoch, nanoseconds
  const double ts[3] = {t0, t0 + 1e5, t0 + 1e6};  // +0.1 ms, +1 ms
  for (uint32_t i = 0; i < n; ++i) {
    size_t base = i * point_step;
    put<float>(data, base + 0, 1.f);
    put<float>(data, base + 4, 2.f);
    put<float>(data, base + 8, 3.f);
    put<double>(data, base + 12, ts[i]);
  }
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid && layout.has_time);
  std::vector<OutPoint> out;
  ASSERT_TRUE(convertCloud(data.data(), data.size(), point_step, n, false,
                           layout, {}, out));
  EXPECT_NEAR(out[0].time_ms, 0.0, 1e-4);
  EXPECT_NEAR(out[1].time_ms, 0.1, 1e-3);
  EXPECT_NEAR(out[2].time_ms, 1.0, 1e-3);
}

TEST(PointCloudAdapter, AutoSniffsUint64AbsoluteMicroseconds) {
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}, {"z", 8, FLOAT32, 1},
      {"time", 12, FLOAT64, 1},
  };
  const uint32_t point_step = 20;
  const uint32_t n = 2;
  std::vector<uint8_t> data(point_step * n, 0);
  const double t0 = 1.7e15;  // epoch microseconds
  const double ts[2] = {t0, t0 + 500.0};  // +0.5 ms
  for (uint32_t i = 0; i < n; ++i) {
    size_t base = i * point_step;
    put<float>(data, base + 0, 1.f);
    put<float>(data, base + 4, 2.f);
    put<float>(data, base + 8, 3.f);
    put<double>(data, base + 12, ts[i]);
  }
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid && layout.has_time);
  std::vector<OutPoint> out;
  ASSERT_TRUE(convertCloud(data.data(), data.size(), point_step, n, false,
                           layout, {}, out));
  EXPECT_NEAR(out[1].time_ms - out[0].time_ms, 0.5, 1e-6);
}

TEST(PointCloudAdapter, AutoKeepsConventionForRelativeOffsets) {
  // Relative float32 seconds (Velodyne-style, values << 5e8): the sniff must
  // NOT fire; the float=seconds convention applies as before.
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}, {"z", 8, FLOAT32, 1},
      {"time", 12, FLOAT32, 1},
  };
  const uint32_t point_step = 16;
  const uint32_t n = 2;
  std::vector<uint8_t> data(point_step * n, 0);
  const float ts[2] = {0.0f, 0.05f};  // 50 ms scan
  for (uint32_t i = 0; i < n; ++i) {
    size_t base = i * point_step;
    put<float>(data, base + 0, 1.f);
    put<float>(data, base + 4, 2.f);
    put<float>(data, base + 8, 3.f);
    put<float>(data, base + 12, ts[i]);
  }
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid && layout.has_time);
  std::vector<OutPoint> out;
  ASSERT_TRUE(convertCloud(data.data(), data.size(), point_step, n, false,
                           layout, {}, out));
  EXPECT_NEAR(out[1].time_ms - out[0].time_ms, 50.0, 1e-3);
}

// ---------------- malformed-input memory-safety guards (hazard 82) ----------
//
// These guards had ZERO automated coverage: deleting the field-fits check in
// convertCloud passed the entire suite, and it is the guard standing between a
// malformed PointCloud2 and an out-of-bounds heap read in readFieldAsDouble.
// The threat model is a misbehaving or hostile publisher, so the check must be
// pinned, not assumed.

TEST(PointCloudAdapterSafety, RejectsFieldExtendingPastPointStride) {
  // z's float32 runs to offset 12, but the declared stride is 10 — reading it
  // walks past the last point's row and off the end of the buffer.
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}, {"z", 8, FLOAT32, 1},
  };
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid);
  std::vector<uint8_t> data(10 * 4, 0);
  std::vector<OutPoint> out;
  EXPECT_FALSE(convertCloud(data.data(), data.size(), /*point_step=*/10,
                            /*num_points=*/4, false, layout, {}, out));
  // Exactly-fitting stride is the boundary and must be ACCEPTED.
  std::vector<uint8_t> ok_data(12 * 4, 0);
  EXPECT_TRUE(convertCloud(ok_data.data(), ok_data.size(), /*point_step=*/12,
                           /*num_points=*/4, false, layout, {}, out));
}

TEST(PointCloudAdapterSafety, RejectsOptionalFieldPastStride) {
  // The optional time/intensity fields go through the same check; a layout
  // where only THEY overrun must still be rejected.
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1},  {"y", 4, FLOAT32, 1},
      {"z", 8, FLOAT32, 1},  {"t", 14, FLOAT32, 1},
  };
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid && layout.has_time);
  std::vector<uint8_t> data(16 * 4, 0);
  std::vector<OutPoint> out;
  EXPECT_FALSE(convertCloud(data.data(), data.size(), /*point_step=*/16,
                            /*num_points=*/4, false, layout, {}, out));
}

TEST(PointCloudAdapterSafety, RejectsUnderDeclaredBuffer) {
  // num_points * point_step must fit in the buffer the publisher handed us.
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}, {"z", 8, FLOAT32, 1},
  };
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid);
  std::vector<uint8_t> data(12 * 3, 0);   // room for 3 points...
  std::vector<OutPoint> out;
  EXPECT_FALSE(convertCloud(data.data(), data.size(), 12, /*num_points=*/4,
                            false, layout, {}, out));   // ...4 declared
  EXPECT_TRUE(convertCloud(data.data(), data.size(), 12, 3, false, layout, {}, out));
}

TEST(PointCloudAdapterSafety, RejectsZeroPointStep) {
  std::vector<FieldInfo> fields = {
      {"x", 0, FLOAT32, 1}, {"y", 4, FLOAT32, 1}, {"z", 8, FLOAT32, 1},
  };
  auto layout = resolveLayout(fields);
  ASSERT_TRUE(layout.valid);
  std::vector<uint8_t> data(48, 0);
  std::vector<OutPoint> out;
  EXPECT_FALSE(convertCloud(data.data(), data.size(), /*point_step=*/0, 4,
                            false, layout, {}, out));
}
