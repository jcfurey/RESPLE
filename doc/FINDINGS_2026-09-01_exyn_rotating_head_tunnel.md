# Exyn rotating-head tunnel LIO evaluation (2026-09-01)

## Executive result

RESPLE-LIO can process the Exyn puck's continuously rotating head correctly
when the corrected cloud is seeded from a complete head revolution while the
platform is stationary. The validated configuration produced stable full-speed
RViz playback through the large bag.

A residual back-and-forth motion remains in the feature-poor middle of the
tunnel. Its velocity and estimator-derived acceleration are synchronized with
the 0.900928 Hz head rotation, while the physical IMU has very little energy at
the same harmonics. This points to a scan-geometry/estimator response rather
than unmodelled physical vehicle motion or a remaining gimbal-transform error.

None of the estimator changes screened in this study improved that residual.
The two strongest candidates made it materially worse, so no estimator code or
new parameter from those experiments was retained. The recommended result is
the known-good rotating-head initialization and reliable input profile below.

## Dataset and replay contract

| Item | Value |
| --- | --- |
| Source bag | `2025_09_09.13_56_34.139.raw-sensors.ros2` |
| Bag start | `1757444203098336006` ns |
| Bag duration | approximately 1105.5 s |
| Evaluated tunnel interval | bag offsets 774.533–891.413 s |
| RESPLE revision | `f89e5c6` on `lyrical` |
| Baseline playback | continuous 1× replay from the stationary beginning |
| Baseline outputs | estimator and Mapping node enabled; RViz enabled |

Replay history is part of the estimator state. A process started near offset
770 s initializes while the platform is moving and builds a different initial
map. That is not equivalent to starting at the stationary beginning and seeking
to the same interval. One nominally unchanged mid-motion run had a 5 s
high-pass position p95 of 1.63376 m, 4.05× the valid baseline's 0.40365 m.

Consequently, a valid quantitative A/B must:

1. Start each process before the stationary beginning of the bag.
2. Replay the complete history into both arms.
3. Use the same playback rate, Mapping/RViz load, QoS, and topic graph.
4. Compare a common timestamp interval after excluding filter edges.

Cold-start trials from the tunnel were useful for debugging, but are not used
as evidence for selecting a tuning change.

## Input-chain validation

The moving-head correction was checked before tuning the estimator:

- The puck rotates about its Z axis at 0.900928 Hz; incremental-axis alignment
  percentiles were all 1.0 over the evaluated interval.
- Corrected `PointCloud2` output was compared with the vendor-derived cloud over
  3.55 million points. The XYZ residual was 0.000939 m median and 0.0117 m p95.
- Selecting the preceding gimbal state for each cloud matches the vendor
  transform convention. Interpolating gimbal poses made the comparison worse.
- Each 10 Hz LiDAR update advances the head by approximately 32.4 degrees, so
  consecutive scans observe substantially different tunnel sectors.

These checks rule out changing the cloud/gimbal time association merely to
make the odometry look smoother.

## Validated baseline

The integration profile uses these material values:

| Parameter | Value | Rationale |
| --- | ---: | --- |
| `sensor_qos_reliable` | `true` | Preserve complete scans during replay |
| `init_map_window_ms` | `1300` | One approximately 1.11 s head revolution plus a complete puck scan |
| `point_filter_num` | `1` | Keep all finite puck returns before voxelization |
| `ds_scan_voxel` | `0.1` m | Retain rotating-sector geometry |
| `ds_lm_voxel` | `0.2` m | Bound local-map density |
| PointCloud2 `w_pt` | `0.01` m² | Established baseline measurement variance |
| `cov_acc` | `[1.0, 1.0, 1.0]` | Established baseline IMU covariance |
| `cov_gyro` | `[0.1, 0.1, 0.1]` | Established baseline IMU covariance |
| `num_points_upd` | `500` | Established baseline update size |
| `range_noise_relative` | `false` | Candidate did not improve this bag |
| `lidar_gate_sigma` | `0.0` | Experimental measurement gate disabled |
| `loc_gate_trans_min_eig` | `0.0` | Experimental localization gate disabled |
| `map_insert_lag_knots` | `0` | Experimental estimator-map lag disabled |

Startup combined 79,258 points over a 1399 ms seed span and voxelized them to
41,735 map points. The filter was then re-anchored at timestamp
`1757444208111317944` ns, after the complete seed boundary.

### Baseline tunnel metrics

Odometry was resampled at 50 Hz. Position slosh is the residual from a 5 s
moving average; velocity is derived from position and high-passed over 2 s.
The local direction of the 5 s trend defines the along- and cross-tunnel axes.
Convolution edges are excluded, and path length is evaluated at 10 Hz.

| Metric | p50 | p95 | Maximum |
| --- | ---: | ---: | ---: |
| Position high-pass norm (m) | 0.12535 | 0.40365 | 0.54182 |
| Velocity high-pass norm (m/s) | 0.36551 | 1.02893 | 3.10982 |
| NIS / degrees of freedom | 0.0214 | 0.0493 | 0.4382 |

| Directional/path metric | Value |
| --- | ---: |
| Along-tunnel position high-pass p95 | 0.39412 m |
| Cross-tunnel position high-pass p95 | 0.14484 m |
| Along-tunnel velocity high-pass p95 | 0.81159 m/s |
| Cross-tunnel velocity high-pass p95 | 0.76329 m/s |
| Path / displacement / excess | 91.9544 / 24.5169 / 67.4375 m |
| Used/candidate ratio p05 / p50 / p95 | 0.3542 / 0.6280 / 0.9040 |
| Translation minimum eigenvalue p05 / p50 / p95 | 0.03753 / 0.06518 / 0.12187 |
| Translation condition number p50 / p95 | 10.71 / 21.40 |

The filter remained `OK` throughout the interval. It recorded 27 covariance
escape admissions, no LiDAR-gate actions, four cycle overruns, and a median
real-time factor of 0.999.

## Rotating-head signature

Welch spectra use the 0.15–10 Hz band. Values below are the fraction of that
band near the first and second gimbal harmonics. The nearest FFT bins to the
0.900928 Hz fundamental and 1.801856 Hz second harmonic were 0.8789 Hz and
1.8066 Hz.

| Signal | First harmonic | Second harmonic |
| --- | ---: | ---: |
| Position high-pass | 11.13% | 1.34% |
| Velocity high-pass | 39.32% | 10.35% |
| Acceleration derived from RESPLE velocity | 29.24% | 21.52% |
| Physical IMU acceleration | 1.79% | 1.45% |

A four-harmonic phase model explains 12.62% of velocity variance and retains
12.12% cross-validated R² when alternating complete head revolutions are held
out. The same model explains only 0.24% of physical IMU acceleration variance
and has negative held-out R². Position itself has much weaker phase locking
because slow tunnel drift dominates it; the repeated signature is clearest in
the correction velocity and derived acceleration.

## Candidate results

### Complete-history quantitative trials

| Candidate | Replay qualification | Position p95 ratio | Velocity p95 ratio | Path-excess ratio | Decision |
| --- | --- | ---: | ---: | ---: | --- |
| Shared LiDAR-batch translation covariance, 2 mm | Continuous 1× from stationary start; Mapping enabled, no RViz | 8.661× | 10.350× | 2.800× | Reject; prototype removed |
| Per-scan relative range weighting | Complete history, 3× pre-roll then 1× tunnel; common offsets 816.243–891.423 s | 1.732× | 1.471× | 2.794× | Reject; leave disabled |

The shared-covariance prototype modeled a rigid translation error common to all
point-to-plane rows in one update. Despite its plausible statistical form, it
destabilized this trajectory: position p95 rose from 0.40365 to 3.49590 m,
velocity p95 rose from 1.02893 to 10.64899 m/s, and covariance-escape admissions
rose from 27 to 679. Its code, parameter, and tests were removed after this
result. The candidate omitted RViz, which reduced rather than increased its
runtime load, so the order-of-magnitude regression cannot be attributed to
RViz overhead.

The range-relative trial is less strictly controlled because its pre-roll rate
differs from the baseline. Over its common interval, however, position p95 rose
from 0.40929 to 0.70909 m, velocity p95 from 0.95876 to 1.40994 m/s, and path
excess from 27.585 to 77.063 m. It is not suitable for this profile.

### Screening trials

The following trials either regressed or failed to show a repeatable benefit
and were reverted: a hard translation-localizability gate at 0.10, a soft gate
with 0.02/0.05 thresholds and 0.25 minimum gain, eight-knot estimator-map
insertion lag, `w_pt` values 0.02 and 0.04, tighter IMU covariance, and a
1000-point update. Several short screening runs were initialized in motion;
they are intentionally not assigned quantitative ratios here.

## Mapping output and RViz

The same valid baseline recording contains both the raw estimator output
`/resple/odom` and the Mapping node's finalized-spline output
`/resple/odometry`. Comparing their common interval with the same high-pass
method gives:

| Metric | `/resple/odom` | `/resple/odometry` |
| --- | ---: | ---: |
| Position high-pass p50 / p95 / max (m) | 0.12531 / 0.40374 / 0.54182 | 0.12231 / 0.40401 / 0.53296 |
| Velocity high-pass p50 / p95 / max (m/s) | 0.36546 / 1.02913 / 3.10982 | 0.22363 / 0.74898 / 1.49389 |
| Path / displacement / excess (m) | 91.7796 / 24.3450 / 67.4347 | 88.0485 / 24.3481 / 63.7005 |

The Mapping output reduces velocity p95 by 27.2% and maximum velocity jitter by
52.0%, but position p95 is effectively unchanged. It is therefore a useful
lower-rate visualization output for RViz, not a fix for estimator position
sloshing and not a drop-in low-latency fusion source.

## Recommendation and next experiment

Keep the validated baseline and always initialize from a stationary interval
covering at least one full head revolution plus one complete puck scan. Do not
enable the rejected range weighting, localization gates, map-insertion lag, or
shared-covariance prototype for this dataset.

The remaining artifact needs a head-aware temporal measurement model rather
than another scalar covariance adjustment. Promising designs are balanced
multi-phase aggregation over a complete revolution or an explicitly modeled
gimbal-phase nuisance term. Either must preserve real periodic vehicle motion
and be validated with complete-history 1× A/B runs. An independent constraint
along the featureless tunnel axis remains the most direct way to resolve true
geometric unobservability.

## Limitations

This is a one-bag empirical smoothness study, not an absolute-accuracy result.
There is no independent tunnel ground truth in the reported A/B metrics, and
ROS scheduling makes separate replays mildly nondeterministic. Conclusions
about the two quantitative regressions are strong because their effects are
large; the qualitative screening results should be repeated under the replay
contract above before being generalized to another platform or sensor.
