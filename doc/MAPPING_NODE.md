# The Mapping node is optional — running RESPLE solo

**TL;DR:** The `Mapping` executable is a *downstream visualization consumer*,
not part of the odometry pipeline. The complete odometry interface —
`odom`, `pose`, `pose_cov`, the `odom → base_link` TF, the deskewed
`current_scan`, diagnostics, and the SaveMap action — is published by the
`RESPLE` node alone, and is unaffected by whether Mapping runs. All launch
files therefore start **RESPLE-only by default**; pass `use_mapping:=true`
to also start Mapping.

```bash
ros2 launch resple resple_ouster.launch.py                    # RESPLE only (default)
ros2 launch resple resple_ouster.launch.py use_mapping:=true  # + global-map visualization
```

## Two processes, one-way dataflow

```text
sensors (LiDAR, IMU)
   │
   ▼
┌─────────────────────────────────────────────┐
│ RESPLE node (estimator)                     │
│  IEKF + B-spline + internal ikd-Tree        │      odom, pose, pose_cov (per knot, e.g. 100 Hz)
│  (the odometry's own matching map)          ├──→   current_scan (deskewed, odom frame, scan rate)
│                                             │      TF: odom → base_link
│                                             │      resple_diagnostics, SaveMap action
└──────────────┬──────────────────────────────┘
               │  est_window (spline window)
               │  start_time (init handshake)
               ▼
┌─────────────────────────────────────────────┐      global_map, traj_path,
│ Mapping node (visualization / map keeping)  ├──→   active_control_points, odometry,
│  re-interpolates the spline, accumulates    │      TF: map → odom
│  a global display cloud                     │
└─────────────────────────────────────────────┘
```

The arrow between the nodes points one way. Mapping subscribes to RESPLE's
`est_window` (the spline window, `estimate_msgs/Estimate`) and `start_time`
(the post-gravity-alignment init handshake), plus the raw sensor topic for
cloud coloring. **RESPLE subscribes to nothing from Mapping.** There is no
feedback path by which Mapping's presence or absence can influence the
estimate.

## Why the odometry is identical without Mapping

Three load-bearing facts:

1. **One-way dataflow.** As above — Mapping is a pure consumer. Killing a
   subscriber cannot change a publisher's output.
2. **The odometry's matching map is internal to RESPLE.** The estimator
   matches scans against its *own* incremental ikd-Tree (a translation-unit
   global inside `RESPLE.cpp`), fed by its own map-update pipeline. The
   global cloud Mapping accumulates is a separate data structure that the
   estimator never queries. Dropping Mapping does **not** drop the
   odometry's map.
3. **The only coupling is host resources.** Both are separate processes
   competing for CPU/memory. Removing Mapping *frees* resources for the
   estimator — it cannot hurt it.

A subtle reinforcement: RESPLE's `publishPoseAndTf()` (odom/pose/TF) is
deliberately decoupled from its own point-cloud/map path, so the odometry
stream keeps flowing at knot rate even when map maintenance is busy. The
odometry output path simply never goes anywhere near a Mapping concern.

## What you keep vs. what you lose

With **RESPLE solo** (`use_mapping:=false`, the default) you keep the full
estimator interface:

| Output | Notes |
| --- | --- |
| `odom`, `pose`, `pose_cov` | Per knot (e.g. 100 Hz at `knot_hz: 100`), IEKF covariance attached |
| TF `odom → base_link` | Gated by `odom/publish_tf` |
| `current_scan` | Deskewed scan in the odom frame, at scan rate |
| `est_window`, `start_time` | Still published (a Mapping node started later just works) |
| `resple_diagnostics`, `/diagnostics` | Estimator health |
| `SaveMap` action | Hosted by **RESPLE** (saves the internal ikd-Tree map), not Mapping |

What you lose is exactly Mapping's outputs:

| Output | Notes |
| --- | --- |
| `global_map` | Accumulated display cloud. Deskewed `map_deskew_lag_knots` (default 8) knots behind the spline edge, so each scan is placed with fully-converged — not bleeding-edge — poses; costs `lag × dt` of display latency and fixes the motion-induced azimuth smear (HARDENING §6.3) |
| `traj_path` | Trajectory history for rviz/Foxglove |
| `active_control_points` | The 4 active B-spline knots |
| `odometry` (Mapping's) | Redundant re-interpolation of the same spline |
| TF `map → odom` | See REP-105 note below |

**REP-105 note:** only Mapping publishes `map → odom` (an identity latch at
init, then live). RESPLE solo roots the TF tree at `odom`. That is the
correct shape for feeding an external mapping/planning stack that builds its
own world representation; but if some consumer requires a `map` frame from
*this* package, run with `use_mapping:=true`.

**map → odom accuracy:** the transform is composed from the spline path tip
(base→map) and RESPLE's odom→base TF, both sampled **at the tip's stamp**
(time-paired exact lookup, with a latest-available fallback during warm-up).
The tip itself is lagged by `map_deskew_lag_knots`, so the composition uses
only fully-converged knots. The odometry path stays bleeding-edge; only the
slow-varying drift-correction frame trades latency for accuracy.

**Coexisting with an external localization stack** (see PARAMETERS.md
"TF ownership" for the full recipe):

- *External odom EKF owns `odom → base`* (RESPLE demoted with
  `odom/publish_tf: false`): Mapping keeps working unchanged — its
  `odom→base` lookup simply returns the **EKF's fused** pose, so the
  composed `map → odom` absorbs the EKF-vs-RESPLE discrepancy (wheel-slip
  corrections etc.). That is exactly the standard drift-correction
  semantics of the `map` frame; nothing to configure.
- *External global localizer / SLAM owns `map → odom`*: set
  `map/publish_tf: false` (Mapping keeps publishing `global_map`,
  `traj_path`, `odometry` — only the TF is ceded), or don't run Mapping.
- *Runtime guard*: both nodes watch `/tf` + `/tf_static` for a foreign
  publisher on the pair they own (`tf_conflict_action` warn/yield;
  `tf_absent_warn_sec` catches "demoted but no external owner ever
  showed up"). Mapping's identity latch is stamp-whitelisted, and in
  `yield` mode the latch is skipped — not consumed — while a foreign
  owner is active.

**Do not feed Mapping's `odometry` topic to a fusion EKF.** Its pose is in
the `map` frame and its covariance is the static `cov_pose`/`cov_twist`
config diagonal — it exists for visualization parity. The fusion input is
the RESPLE node's `odom` topic, whose covariance is the live IEKF
posterior.

## Empirically verified

HelmDyn01 bag, LIO mode (`if_lidar_only: false`), ROS 2 Jazzy, 2026-06-10,
RESPLE + rviz + static TF only — no Mapping process:

| Check | Result |
| --- | --- |
| `odom` rate | 100.7 Hz (= knot rate) |
| `current_scan` rate | 9.5 Hz (= scan rate) |
| TF `odom → base_link` | live and updating |
| Estimator buffer (`pc_buff`) | 0 — worker keeping up |
| Gravity init / IEKF | nominal |

(If `global_map` / `traj_path` still appear in `ros2 topic list`, that is
rviz holding subscriptions — they have no publisher without Mapping.)

## Feeding an external mapping stack (octomap, vdb_mapping, …)

Run RESPLE solo and consume:

- **`odom` + the `odom → base_link` TF** — the LIO output, and either
- **`current_scan`** — already deskewed into the odom frame, or
- the **raw sensor topic + TF** if you prefer to deskew/integrate inside
  your own stack.

## Scope

The `use_mapping` argument (default `false`) is wired into all 16 launch
files (every `resple_*.launch.py` that previously started Mapping;
`mapping_delay` still applies when enabled). Scripts that exec the binaries
directly (accuracy / sanitizer-replay / benchmark harnesses) are unaffected;
they start Mapping deliberately so its code paths stay exercised — the
accuracy trajectory itself is recorded from RESPLE's `/odom`.
