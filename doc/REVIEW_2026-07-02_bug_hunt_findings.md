# RESPLE Bug Hunt & Logic Verification — Findings (INTERIM)

**Date:** 2026-07-02  ·  **Status: INTERIM — pre-verification snapshot.**
An adversarial 3/2/1-lens verification pass (scaled by severity) plus a
completeness critic are still running; every finding below is a *candidate*
until its verdict lands. This file will be updated in place with verdicts
(CONFIRMED / PLAUSIBLE / REFUTED) when the pipeline completes.

## Method

Eleven independent finder agents, each auditing one dimension of the package
against reference material:

- **Academic:** RESPLE paper (arXiv:2504.11580); Sommer et al. 2020,
  *Efficient Derivative Computation for Cumulative B-Splines on Lie Groups*
  (arXiv:1911.08860); X-ICP (arXiv:2211.16335); standard IEKF/Joseph-form theory.
- **Upstream repos:** ASIG-X/RESPLE at the vendor point (git 3d0e78c, extracted
  from this repo's history); current hku-mars/ikd-Tree main; hku-mars/FAST_LIO
  (esti_plane / local-map FOV conventions).
- **Local docs treated as known-issue list:** CLAUDE.md hazards table (38 fixed
  hazards), HARDENING.md, doc/REVIEW_2026-06-27_dliio_lessons_and_bugs.md (A1-A4).
  Finders were instructed not to re-report fixed issues — but to flag a fix that
  is itself wrong (several findings below are exactly that).

Dimensions: spline math, IEKF core, association/deskew, IMU path, node logic,
mapping node, ikd-Tree, recent commits, config/units, numerics/Eigen,
fresh-code concurrency. 39 raw findings -> 30 after cross-finder dedup.

## Summary table

Note: #1 and #2 are the same underlying defect (four finders converged on it
independently; cross-finder dedup left both entries). They will be merged in
the final verified report.

| # | Sev | Conf | Location | Title | Finder(+corroborated) |
|---|-----|------|----------|-------|-----------------------|
| 1 | critical | 0.93 | `resple/include/Estimator.h:502` | getLastTwistCovariance rotates by an UNINITIALIZED quaternion: itpQuaternion never writes *q_out when J_q==... | numerics +spline-math |
| 2 | high | 0.92 | `resple/include/Estimator.h:502` | getLastTwistCovariance rotates velocity covariance by an UNINITIALIZED quaternion: itpQuaternion never writ... | iekf +recent-commits |
| 3 | high | 0.8 | `resple/src/RESPLE.cpp:611` | LiDAR extrinsic applied twice (TF in callback + YAML q_bl/t_bl in pointBodyToWorld): net extrinsic silently... | config-units +association,node-logic |
| 4 | high | 0.8 | `resple/src/RESPLE.cpp:3053` | initialization() calls pt_buff.front() on a deque its own stale-backlog drop may have just emptied (UB/SIGS... | node-logic |
| 5 | high | 0.75 | `resple/launch/resple_ntu_day_01_ouster.launch.py:57` | NTU/KTH dataset launch files publish no base_link->sensor TF, and the TF gate silently drops every scan: no... | config-units |
| 6 | high | 0.72 | `resple/src/Mapping.cpp:320` | Mapping transformPoint builds the map in the wrong frame: it re-applies the inverse TF lidar extrinsic that... | mapping-node |
| 7 | high | 0.7 | `resple/include/ikd-Tree/ikd_Tree.cpp:1382` | Residual ABBA deadlock: Push_Down's rebuild branch takes working_flag_mutex while search path holds search_... | ikdtree |
| 8 | high | 0.55 | `resple/include/ikd-Tree/ikd_Tree.cpp:291` | Rebuild swap and old-subtree free run without working_flag_mutex when the flattened subtree is empty (hazar... | ikdtree |
| 9 | medium | 0.85 | `resple/src/RESPLE.cpp:559` | Worker heartbeat reads callback-mutated pc_buff.size() without mtx_pc — hazard-36-class data race left behi... | concurrency-fresh |
| 10 | medium | 0.75 | `resple/src/RESPLE.cpp:2142` | transformImu mixes units when acc_ratio=true: centripetal lever-arm correction gets scaled by 9.81x | imu-path |
| 11 | medium | 0.72 | `resple/src/RESPLE.cpp:2877` | last_pose_pub_time_ns_ never reset on lifecycle cleanup — pose/odom/TF silently suppressed after a re-cycle... | node-logic +concurrency-fresh |
| 12 | medium | 0.7 | `resple/include/utils/geometry_core.h:67` | fitPlane solves the plane fit via float32 normal equations (ATA), squaring the condition number — normals d... | association |
| 13 | medium | 0.7 | `resple/src/Mapping.cpp:193` | processScan's deskew-lag gate and drop-old test use points.back() of a VoxelGrid-filtered cloud as 'scan en... | mapping-node |
| 14 | medium | 0.7 | `resple/include/ikd-Tree/ikd_Tree.cpp:1661` | Points_deleted / Multithread_Points_deleted grow without bound: acquire_removed_points is never called and ... | ikdtree |
| 15 | medium | 0.7 | `doc/PARAMETERS.md:93` | q_lb/t_lb documented as 'LiDAR->body' but the code convention is body(IMU)->lidar — doc-following users sup... | config-units |
| 16 | medium | 0.65 | `resple/include/SplineState.h:239` | updateKnots appends across index gaps at the wrong deque position, permanently shifting the replica spline'... | mapping-node |
| 17 | medium | 0.6 | `resple/src/RESPLE.cpp:2007` | Q_block_new written to the bias block via bottomRightCorner on the 30x30 Q: newest RCP gets zero process no... | imu-path |
| 18 | medium | 0.6 | `resple/src/RESPLE.cpp:2220` | Gravity alignment and all pre-init IMU samples bypass the base_link extrinsic: initial orientation is wrong... | imu-path |
| 19 | medium | 0.6 | `resple/include/ikd-Tree/ikd_Tree.cpp:1361` | Data race on non-atomic Rebuild_Ptr read in searcher-path Push_Down (double-load can null-deref) | ikdtree |
| 20 | medium | 0.6 | `resple/include/utils/point_cloud_adapter.h:212` | Generic PointCloud2 time_unit 'auto' misclassifies float64 nanosecond timestamp fields (livox_ros_driver2 c... | config-units |
| 21 | low | 0.9 | `doc/PARAMETERS.md:81` | PARAMETERS.md documents cov_pose/cov_twist defaults as [0.1 x6]; the code default is {0.2, 0.2, 0.2, 0.1, 0... | config-units |
| 22 | low | 0.7 | `resple/include/Estimator.h:877` | NIS dof counts zeroed (gate-rejected / outlier-clamped) measurement rows, biasing the windowed NIS/dof cons... | spline-math |
| 23 | low | 0.7 | `resple/src/MapSaving.cpp:78` | MapSaving service callback lets pcl::IOException escape — save with unwritable pcd_save_path aborts the nod... | concurrency-fresh +mapping-node,recent-commits |
| 24 | low | 0.65 | `resple/src/RESPLE.cpp:3105` | Pre-init imu_buff is unbounded and the gravity all-windows variance scan is O(buff_size x n_imu) per 50 ms ... | node-logic |
| 25 | low | 0.6 | `resple/include/SplineState.h:163` | setIdles chains q_idle / q_knots[0] with an off-by-one idle-delta convention, inconsistent with the 'delta ... | iekf +association,numerics |
| 26 | low | 0.6 | `resple/src/RESPLE.cpp:2062` | Single cached lidar_to_baselink_ / have_lidar_transform_ shared across all LiDARs: in multi-LiDAR (MLO/MLIO... | association |
| 27 | low | 0.55 | `resple/include/utils/point_cloud_ingest.h:90` | Generic PointCloud2 ingestion ignores row_step: organized clouds with padded rows are misparsed | config-units |
| 28 | low | 0.55 | `resple/src/RESPLE.cpp:1725` | Omitting ds_scan_voxel / nn_thresh (documented as optional) yields degenerate 0.0 defaults: zero voxel leaf... | config-units |
| 29 | low | 0.55 | `resple/include/Estimator.h:574` | Loc-gate covariance-inflation accumulator survives filter re-initialization — a fresh run publishes the pre... | concurrency-fresh |
| 30 | low | 0.5 | `resple/src/RESPLE.cpp:3070` | LO mode initialization now hard-blocks forever without an IMU topic (regression vs upstream LiDAR-only init) | imu-path |

## Findings

### 1. [CRITICAL] getLastTwistCovariance rotates by an UNINITIALIZED quaternion: itpQuaternion never writes *q_out when J_q==nullptr but J_w!=nullptr

- **Location:** `resple/include/Estimator.h:502`
- **Found by:** numerics, spline-math  ·  **Finder confidence:** 0.93
- **Verification:** _pending_

**Description.** Estimator::getLastTwistCovariance() calls `spl.itpQuaternion(t, &q_out, &w_out, nullptr, &J_w)` (Estimator.h:502) with q_out requested, J_q=nullptr, J_w non-null. In SplineState::itpQuaternion, the `(J_q || J_w)` branch is taken, but the ONLY assignment to *q_out in that branch is inside `if (J_q)` (SplineState.h:429 `*q_out = q_itps[3];`); the `else` branch that would also assign q_out (SplineState.h:492-499) is never reached. So q_out — a default-constructed Eigen::Quaterniond, which Eigen leaves UNINITIALIZED (and EIGEN_INITIALIZE_MATRICES_BY_NAN is Debug-only per CMakeLists) — is consumed at Estimator.h:521 `const Eigen::Matrix3d R = q_out.toRotationMatrix();` and used to rotate the world-frame linear-velocity covariance into the body frame (P_v_body = R^T P_vw R). getLastTwistCovariance is called on every publish in publishPoseAndTf (RESPLE.cpp:2905-2907, both LO and LIO default paths), so the odometry message's twist.covariance linear-velocity 3x3 block is computed from stack garbage every frame: nondeterministic, unnormalized (toRotationMatrix of a non-unit quaternion scales by |q|^2), potentially huge/NaN. The downstream odom EKF (robot_localization odom1 input per CLAUDE.md) fuses body-frame velocity weighted by exactly this block. The angular block (Jw path) and the pose covariance (getLastPoseCovariance passes a non-null J_q) are unaffected. This is a locally-added function (not upstream) violating upstream itpQuaternion's implicit contract; it is not among CLAUDE.md hazards 1-38 nor REVIEW A1-A4.

**Evidence.**

```
Estimator.h:500-502: `Eigen::Quaterniond q_out; Eigen::Vector3d w_out; spl.itpQuaternion(t, &q_out, &w_out, nullptr, &J_w);` then line 521: `const Eigen::Matrix3d R = q_out.toRotationMatrix();`. SplineState.h:407 `if (J_q || J_w) {` ... line 423 `if (J_q) { ... *q_out = q_itps[3]; }` — no q_out write outside that guard in this branch. Empirically confirmed with a minimal repro compiled against the real SplineState.h + test stubs: q_out coefficients pre-poisoned with {1e300,-3.14,42,7.7} survive the call unchanged (output: 'BUG: q_out NOT written by itpQuaternion'), while the no-Jacobian path returns the correct quaternion (0.138,0.069,0.208,0.966).
```

**Reference.** Upstream refs/upstream_SplineState.h:229-274 has the same structure (q_out written only under if(J_q)), but upstream has no caller with the (q_out, J_q=null, J_w!=null) combination — upstream callers are prepIMU (both Jacobians non-null) and plain q queries. getLastTwistCovariance is a workspace-local addition (local_vs_upstream.diff) that introduced the mismatched call.

**Suggested fix.** Either (a) in SplineState::itpQuaternion, compute the q_itps chain and assign `*q_out` whenever `q_out && !J_q` inside the Jacobian branch (q_delta_scale[0..3] are already computed there), or (b) in getLastTwistCovariance pass a throwaway Jacobian43 as J_q so the existing q_out path runs: `Jacobian43 J_q_dummy; spl.itpQuaternion(t, &q_out, &w_out, &J_q_dummy, &J_w);`. Add a regression test asserting q_out is written for every non-null-pointer combination.

### 2. [HIGH] getLastTwistCovariance rotates velocity covariance by an UNINITIALIZED quaternion: itpQuaternion never writes q_out when J_q==nullptr but J_w!=nullptr

- **Location:** `resple/include/Estimator.h:502`
- **Found by:** iekf, recent-commits  ·  **Finder confidence:** 0.92
- **Verification:** _pending_

**Description.** SplineState::itpQuaternion has two output paths: in the Jacobian branch (entered when J_q || J_w), *q_out is assigned only inside `if (J_q)` (SplineState.h:423-429); the no-Jacobian else branch writes it under `if (q_out)`. The locally-added Estimator::getLastTwistCovariance calls `spl.itpQuaternion(t, &q_out, &w_out, nullptr, &J_w)` — J_q is nullptr and J_w is non-null, so the Jacobian branch runs and q_out (a default-constructed Eigen::Quaterniond, which Eigen leaves UNINITIALIZED) is never written. Line 521 then computes `R = q_out.toRotationMatrix()` from stack garbage and uses it to rotate the world-frame linear-velocity covariance into the body frame: `P_v_body = R.transpose() * P_vw * R`. Since R is built from an arbitrary non-unit quaternion it has arbitrary scale (~|q|^2 per axis) and orientation, so the published twist linear covariance is garbage (arbitrarily scaled, possibly enormous or NaN if the stack holds NaN patterns). This runs on every publishPoseAndTf cycle (RESPLE.cpp:2905-2907, 2940-2942) and the result is published in /localization/resple/odometry twist.covariance, feeding the downstream robot_localization odom EKF whenever RESPLE is enabled (the supported one-line-revert LIO config). The angular block (Jw·P·Jwᵀ) and the twist values themselves are unaffected; only the linear 3x3 covariance block is corrupted. Empirically confirmed: compiled a standalone repro against SplineState.h — after `itpQuaternion(t, &q, &w, nullptr, &J_w)` the pre-poisoned q (1234.5, -7, 8, 9) was returned unchanged, and toRotationMatrix() gave entries like -289.0. Not among CLAUDE.md hazards 1-38 nor A1-A4.

**Evidence.**

```
Estimator.h:499-502: `Jacobian33 J_w; Eigen::Quaterniond q_out; Eigen::Vector3d w_out; spl.itpQuaternion(t, &q_out, &w_out, nullptr, &J_w);` then Estimator.h:521-522: `const Eigen::Matrix3d R = q_out.toRotationMatrix(); const Eigen::Matrix3d P_v_body = R.transpose() * P_vw * R;`. SplineState.h:407 `if (J_q || J_w) {` ... 423 `if (J_q) {` ... 429 `*q_out = q_itps[3];` — the only q_out write in this branch is gated on J_q, which is nullptr here. Repro run output: "q after call: 1234.500000 -7.000000 8.000000 9.000000" vs reference (0.978813, 0.178727, 0.089363, -0.044682); "R(0,0)=-289.000000". Consumer: RESPLE.cpp:2905-2907 `const Eigen::Matrix<double, 6, 6> P_twist = ... getLastTwistCovariance();` and 2940-2942 copy it into odom_msg.twist.covariance.
```

**Reference.** Upstream contract: refs/upstream_SplineState.h:247-253 — *q_out is likewise written only under `if (J_q)` in the Jacobian branch; upstream never calls itpQuaternion with (q_out set, J_q null, J_w set), so the trap was latent until the local getLastTwistCovariance addition (not present in refs/upstream_Estimator.h). Eigen docs: Quaternion default constructor leaves coefficients uninitialized.

**Suggested fix.** In SplineState::itpQuaternion's Jacobian branch, honor q_out independently of J_q: compute the q_itps chain (or `cp0*qds0*qds1*qds2*qds3`, normalized) and write `*q_out` whenever q_out != nullptr. Alternatively (minimal, caller-side): in getLastTwistCovariance obtain q_out from a separate `spl.itpQuaternion(t, &q_out)` call, or pass a dummy Jacobian43 so the J_q branch runs. Add a regression test calling itpQuaternion(t, &q, &w, nullptr, &J_w) and comparing q against the plain-path result.

### 3. [HIGH] LiDAR extrinsic applied twice (TF in callback + YAML q_bl/t_bl in pointBodyToWorld): net extrinsic silently cancels; two-LiDAR heap config fuses clouds in mutually rotated frames

- **Location:** `resple/src/RESPLE.cpp:611`
- **Found by:** config-units, association, node-logic  ·  **Finder confidence:** 0.8
- **Verification:** _pending_

**Description.** Every sensor callback now pre-transforms the cloud into base_link via TF (e.g. RESPLE.cpp:2474/2803 pcl::transformPointCloud(..., lidar_to_baselink_)), but processData still constructs PointData with the YAML extrinsic (line 611: 'PointData pt(pc_last_ds->points[i], time_begin, lidar.q_bl, lidar.t_bl, ...)') and Association::pointBodyToWorld (Association.h:96) applies q_bl/t_bl AGAIN: p_global = q * (q_bl*p_body + t_bl) + pos. Upstream applied q_bl/t_bl to RAW lidar-frame points (upstream_RESPLE.cpp has no TF transform at all), so applying it to already-base_link points is a second application. The shipped dataset launches set the static TF equal to (q_lb, t_lb) verbatim (e.g. resple_heap launch hesai_tf qx/qy/qz/qw = config hesai q_lb reordered; resple_r_campus/helmdyn/nc_short TF translation == YAML t_lb), so the composition q_bl*(q_lb*p_l + t_lb) + t_bl collapses to exactly p_l — the extrinsic is silently NULLED and each cloud enters the estimator in its own raw sensor frame. Single-LiDAR datasets get a lidar-frame trajectory with an IMU/LiDAR frame mismatch in LIO (nc_short's q_lb=[0,0,0,1] is a 180-deg yaw that the IMU path does not see — updateImuTransform falls back to pass-through); the two-LiDAR heap MLO/LO config is worse: livox and hesai are physically ~90 deg apart (hesai q_lb w~0.0008), and with both nets collapsing to identity the two clouds are inserted into ONE map in frames rotated 90 deg from each other. The Mapping node explicitly avoids this (Mapping.cpp:313-320: 'Applying lidar->IMU (lidar.q_bl/t_bl) here would double-count the extrinsic') and composes TF*imu_to_lidar correctly (Mapping.cpp:281-284), so RESPLE and Mapping also disagree by q_bl*t_lb whenever the extrinsic is non-identity.

**Evidence.**

```
RESPLE.cpp:611 'PointData pt(pc_last_ds->points[i], time_begin, lidar.q_bl, lidar.t_bl, lidar.w_pt, lidar.sensor_origin_body);' after RESPLE.cpp:2474 'pcl::transformPointCloud(*pc_last, *pc_last, lidar_to_baselink_);' and Association.h:96 'p_global = q.cast<float>() * (q_bl.cast<float>() * p_body + t_bl.cast<float>()) + pos.cast<float>();'. Launch resple_heap_testsite_hoenggerberg.launch.py:63-65 publishes base_link->hesai_lidar with qx=-0.703743110426531 qy=0.710453977622183 — identical to config_heap yaml hesai q_lb [0.000793920391926, -0.703743110426531, 0.710453977622183, -0.000387167175118] (w,x,y,z). q_bl=q_lb.inverse() (common_utils.h:336), so net rotation = q_lb^-1 * q_lb = I: extrinsic cancelled. Mapping.cpp:317 comment: 'Applying lidar->IMU (lidar.q_bl/t_bl) here would double-count the extrinsic, since the point is no longer in the lidar frame.' — RESPLE.cpp does exactly that.
```

**Reference.** upstream_RESPLE.cpp (vendor point 3d0e78c): no transformPointCloud/updateLidarTransform anywhere; PointData gets q_bl/t_bl applied to raw lidar-frame points. Local Mapping.cpp:281-320 shows the intended correct composition.

**Suggested fix.** In RESPLE.cpp processData, since points are already in base_link, pass the base_link->IMU transform (identity when base_link is the estimation frame) into PointData instead of lidar.q_bl/t_bl — mirroring Mapping.cpp transformPoint. Fix the dataset launch TFs to publish lidar-in-body (the inverse of q_lb/t_lb), or set YAML extrinsics to identity when the TF carries them.

### 4. [HIGH] initialization() calls pt_buff.front() on a deque its own stale-backlog drop may have just emptied (UB/SIGSEGV)

- **Location:** `resple/src/RESPLE.cpp:3053`
- **Found by:** node-logic  ·  **Finder confidence:** 0.8
- **Verification:** _pending_

**Description.** The pre-init 'discard stale backlog' block (lines 3012-3050, a workspace-local addition not in upstream) computes a single global cutoff = latest_t_ns - 200ms across ALL lidars and then pops every pt_buff entry older than that cutoff. The non-empty guard for pt_buff runs at lines 2993-2997, BEFORE the drop, and emptiness is never re-checked. Immediately after the drop, line 3053 dereferences lidar_data.pt_buff.front() for every lidar. In a multi-lidar (MLO/MLIO, supported by the 'lidars' list param and upstream modes) configuration where one lidar lags more than 200 ms behind the freshest one (driver hiccup, decimated topic, low-rate sensor), that lidar's entire pt_buff is dropped and front() is called on an empty Eigen::aligned_deque — undefined behavior (garbage start_t_ns anchoring the spline, or SIGSEGV; the worker's try/catch cannot catch UB). A single lidar with scan period > 200 ms (< 5 Hz) can also trigger it when a new scan lands in t_buff between the drain and initialization().

**Evidence.**

```
Lines 3029-3034: `while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns < cutoff) { lidar_data.pt_buff.pop_front(); ... }` — can empty pt_buff entirely for a lagging lidar (cutoff derives from the max over ALL lidars at lines 3014-3025). Then lines 3051-3054: `int64_t start_t_ns = std::numeric_limits<int64_t>::max(); for (const auto& [lidar_name, lidar_data] : lidars_data) { start_t_ns = std::min(start_t_ns, std::max(lidar_data.pt_buff.front().time_ns, int64_t(0))); }` with no emptiness re-check. The only guard (lines 2993-2997 `if (lidar_data.pt_buff.empty()) return false;`) executes before the drop.
```

**Reference.** Upstream refs/upstream_RESPLE.cpp:607-637 has no backlog-drop block — its pt_buff.front() use is always covered by the preceding empty() gate; the local drop block (introduced with the 'start near real time' feature) broke that invariant. Not among CLAUDE.md hazards 1-38 or A1-A4.

**Suggested fix.** After the drop loops, re-check every pt_buff: `for (auto& [n, d] : lidars_data) if (d.pt_buff.empty()) return false;` (or never drop the last retained entry per lidar, e.g. use a per-lidar cutoff min(global_cutoff, pt_buff.back().time_ns)).

### 5. [HIGH] NTU/KTH dataset launch files publish no base_link->sensor TF, and the TF gate silently drops every scan: node never initializes, zero odometry, no warning

- **Location:** `resple/launch/resple_ntu_day_01_ouster.launch.py:57`
- **Found by:** config-units  ·  **Finder confidence:** 0.75
- **Verification:** _pending_

**Description.** resple_ntu_day_01_ouster.launch.py:57, resple_ntu_day_01_livox.launch.py:57 and resple_kth_day_06_ouster.launch.py:57 publish only the upstream leftover static TF ['0','0','0','0','0','0','map','my_frame'] — no base_link->(cloud frame_id) transform. The locally-added updateLidarTransform gate (RESPLE.cpp:2060-2100) is called at the top of every LiDAR callback and returns false (dropping the whole scan) until base_link->cloud-frame resolves. Worse, the _frameExists pre-check (RESPLE.cpp:2067-2069) returns false BEFORE the throttled 'Waiting for LiDAR transform' warning, and 'base_link' only ever enters the TF tree via RESPLE's own odom->base_link broadcast, which happens post-initialization — which needs scans. So these three launches deadlock silently: all scans dropped forever, no odometry, no log line explaining why. Their configs carry non-identity extrinsics (ntu ouster q_lb ~180deg roll) that were previously honored by the upstream code path without any TF.

**Evidence.**

```
resple_ntu_day_01_ouster.launch.py:57 "arguments=['0', '0', '0', '0', '0', '0', 'map', 'my_frame', ...]" is the only static_transform_publisher in the file. RESPLE.cpp:2067-2069: 'if (!tf_buffer_->_frameExists(this->frame_id) || !tf_buffer_->_frameExists(source_frame_id)) { return false; }' (silent), and each callback: 'if(!updateLidarTransform(...->header.frame_id)) return;' (e.g. RESPLE.cpp:2442). frame_id defaults to base_link (RESPLE.cpp:1708); nothing in these launches publishes it.
```

**Reference.** Compare resple_ouster.launch.py:55-62 / resple_r_campus.launch.py:52-59, which DO publish base_link->sensor static TFs; upstream_RESPLE.cpp has no updateLidarTransform gate, so the upstream 'map->my_frame' boilerplate was harmless at the vendor point.

**Suggested fix.** Add a base_link->(bag cloud frame_id) static_transform_publisher to the three dataset launches (per finding 1, with the correctly inverted extrinsic), and move the throttled 'Waiting for LiDAR transform' warning ahead of the _frameExists early-return so a missing TF is at least visible.

### 6. [HIGH] Mapping transformPoint builds the map in the wrong frame: it re-applies the inverse TF lidar extrinsic that the estimator never uses

- **Location:** `resple/src/Mapping.cpp:320`
- **Found by:** mapping-node  ·  **Finder confidence:** 0.72
- **Verification:** _pending_

**Description.** The estimator's spline is trained on base_link-frame points: every RESPLE.cpp sensor callback pre-transforms the cloud via TF (pcl::transformPointCloud(*pc_last, *pc_last, lidar_to_baselink_), RESPLE.cpp:2474) and Association::pointBodyToWorld then applies only the YAML extrinsic (q_bl*p + t_bl, Association.h:96; PointData built with lidar.q_bl/t_bl at RESPLE.cpp:611). So the frame the spline transforms is F_est(p_base) = q_bl*p_base + t_bl = M^{-1}(p_base), where M = Translation(t_lb)*q_lb. The Mapping node's clouds are likewise pre-transformed to base_link, but transformPoint maps them p_imu = baselink_to_imu_ * p_base where baselink_to_imu_ = (lidar_to_baselink_ * M)^{-1} (Mapping.cpp:281-284), i.e. F_map = M^{-1} ∘ L^{-1} with L = the TF lidar→base_link extrinsic. F_map == F_est only when L is identity. In the workspace's production-style configs the YAML is deliberately identity and the whole extrinsic lives in TF (config_06042026.yaml header: "EXTRINSICS COME FROM TF, NOT q_lb/t_lb"), so every /global_map point is first mapped base_link→lidar and then splined: the accumulated map is rigidly displaced (and, for a rotated mount such as an Ouster os_lidar frame, rotated) by the full inverse mount extrinsic relative to the estimator's own world points (/current_scan, the ikd-tree map, and the traj_path/odometry poses published by this same node). The in-code comment claiming that applying q_bl/t_bl "would double-count the extrinsic" is backwards: pointBodyToWorld applies exactly q_bl/t_bl to the identically pre-transformed base_link cloud. The upstream dataset configs are unaffected only because there L is identity (extrinsic in q_lb, TF identity), which is why benchmark replays never surfaced it.

**Evidence.**

```
Mapping.cpp:319-321: `Eigen::Vector3d p_body(pt_in.x, pt_in.y, pt_in.z); Eigen::Vector3d p_imu(baselink_to_imu_ * p_body); Eigen::Vector3d p_global(q_itp * p_imu + t_itp);` with Mapping.cpp:281-284: `Eigen::Affine3d imu_to_lidar = translation * lidar.q_lb; imu_to_baselink_ = lidar_to_baselink_ * imu_to_lidar; baselink_to_imu_ = imu_to_baselink_.inverse();`. Estimator reference path: RESPLE.cpp:2474 `pcl::transformPointCloud(*pc_last, *pc_last, lidar_to_baselink_);` then Association.h:96 `p_global = q * (q_bl * p_body + t_bl) + pos` with q_bl/t_bl from YAML (LidarConfig, common_utils.h:333-337: q_bl = q_lb.inverse(), t_bl = q_lb.inverse()*(-t_lb)), so F_est = M^{-1}(p_base) while F_map = M^{-1}(L^{-1}(p_base)) — an extra inverse-TF factor.
```

**Reference.** Upstream refs/upstream_Mapping.cpp:83-85 (`p_imu = lidar.q_bl * p_body + lidar.t_bl` applied to the SAME cloud the estimator consumes — no TF factor) and refs/upstream_RESPLE.cpp Association convention; local estimator path RESPLE.cpp:611/852/2474 + Association.h:96; config_06042026.yaml:11-20 (extrinsics from TF, q_lb/t_lb identity).

**Suggested fix.** In transformPoint, apply exactly the estimator's operation to the base_link point: p_spline = lidar.q_bl * p_body + lidar.t_bl (identity under the TF-based configs), i.e. drop the lidar_to_baselink_ factor from the chain — replace baselink_to_imu_ with the affine (Translation(t_lb)*q_lb).inverse() alone, and delete the misleading comment. Add a parity test comparing Mapping's transformPoint against Association::pointBodyToWorld with a non-identity lidar_to_baselink_.

### 7. [HIGH] Residual ABBA deadlock: Push_Down's rebuild branch takes working_flag_mutex while search path holds search_rw_mutex_ shared (hazard 34 fix incomplete)

- **Location:** `resple/include/ikd-Tree/ikd_Tree.cpp:1382`
- **Found by:** ikdtree  ·  **Finder confidence:** 0.7
- **Verification:** _pending_

**Description.** The Phase 2.5 #2 fix for hazard 34 removed search_rw_mutex_ from the five mutating fast paths but explicitly kept the shared lock on the search functions (Nearest_Search line 452, Box_Search 476, Radius_Search 485, flatten_safe in ikd_Tree.h:399-402). All of those call Push_Down, whose rebuild branch still does a blocking pthread_mutex_lock(&working_flag_mutex) at lines 1382 and 1436 (the Phase 2.4 comment at lines 1322-1324 says this handling was 'preserved', analyzing only the node->working_flag order, not search_rw->working_flag). The rebuild thread takes working_flag_mutex first (line 262, and holds it from line 295 through the swap) and then search_rw_mutex_ UNIQUE (lines 282 and 316). So: search thread holds search_rw shared, blocks on working_flag in Push_Down; rebuild thread holds working_flag, blocks on search_rw unique waiting for the shared holder -> permanent deadlock. This is exactly the inversion class hazard 34 fixed for the mutators, left in place on the reader side. Reachability in the default LIO config: flatten (line 1641) calls Push_Down on every node including tree_deleted ones, and each Push_Down re-creates pending need_push_down flags on the children, so flatten_safe (called from the async map-publish path, RESPLE.cpp:1142/1651/3239) descending through a lasermapFovSegment-deleted region whose interior contains the current *Rebuild_Ptr target hits the rebuild branch while the rebuild thread is between its working_flag acquisition and its search_rw-unique acquisition. FOV box deletes are precisely what trips the delete criterion and schedules background rebuilds, so the two coincide. The async lambda then hangs holding mtx_map_ unique, the IEKF worker blocks on mtx_map_ shared, and the node goes permanently silent.

**Evidence.**

```
Push_Down else-branch (search path, no working_flag held by caller): line 1381-1382 'else { pthread_mutex_lock(&working_flag_mutex);' (same at 1436). Callers hold search_rw shared: Nearest_Search line 452 'std::shared_lock<std::shared_mutex> rlock(search_rw_mutex_); Search(Root_Node, ...)'; flatten_safe (ikd_Tree.h:400) 'std::shared_lock<std::shared_mutex> rlock(search_rw_mutex_); flatten(Root_Node, ...)'. Opposite order in multi_thread_rebuild: line 262 'pthread_mutex_lock(&working_flag_mutex);' then line 282 / 316 'std::unique_lock<std::shared_mutex> wlock(search_rw_mutex_);'. A shared_mutex unique acquisition blocks until all shared holders release; the shared holder is blocked on working_flag held by the rebuild thread -> cycle.
```

**Reference.** Hazard 34 in /home/user/RESPLE/CLAUDE.md (claims the working_flag<->search_rw inversion fixed by dropping the shared lock from mutating paths only); upstream vendor point refs/upstream_ikd_Tree.cpp Search/Push_Down had no search_rw_mutex_ at all (used search_mutex_counter, which never blocks while holding a reader lock the rebuild thread needs).

**Suggested fix.** In Push_Down's rebuild branch, replace the blocking lock with pthread_mutex_trylock(&working_flag_mutex); on failure, apply the child writes without logging is unsafe, so instead skip the push-down for this node (leave root->need_push_down_to_* set and return; the flags are re-examined on the next visit). Alternatively make searchers never take working_flag: have the rebuild thread clear/set Rebuild_Ptr inside the search_rw unique section so searchers can decide the branch under their shared lock without working_flag.

### 8. [HIGH] Rebuild swap and old-subtree free run without working_flag_mutex when the flattened subtree is empty (hazard 35 fix incomplete)

- **Location:** `resple/include/ikd-Tree/ikd_Tree.cpp:291`
- **Found by:** ikdtree  ·  **Finder confidence:** 0.55
- **Verification:** _pending_

**Description.** multi_thread_rebuild unlocks working_flag_mutex at line 287 after flattening, and only re-acquires it at line 295 INSIDE 'if (int(Rebuild_PCL_Storage.size()) > 0)'. If the rebuild target subtree flattens to zero live points, the subtree swap (lines 315-346: father_ptr->left/right_son_ptr write, Root_Node update, ancestor Update() walk), the 'Rebuild_Ptr = nullptr' write (347) and delete_tree_nodes(&old_root_node) (351) all execute WITHOUT working_flag_mutex. The Phase 2.5 #1 fix (hazard 35) makes mutator-vs-rebuild exclusion depend entirely on the mutators' whole-op working_flag_mutex plus the rebuild thread holding it across the father_ptr read + swap + ancestor walk — this path bypasses that entirely, and Phase 2.5 #2 removed search_rw from the mutating paths, so nothing excludes a concurrent Add_Points/Delete_Point_Boxes: it can read the stale child pointer, descend into the old subtree, and race the swap and the free (UAF), and its ancestor Update() calls race the rebuild's ancestor walk. The empty case is reachable in the default LIO config: Rebuild_Ptr requires TreeSize >= 1500 at scheduling, and lazy box deletion (lasermapFovSegment's Delete_Point_Boxes full-cover branch) marks points deleted WITHOUT reducing TreeSize and without clearing Rebuild_Ptr, so a scheduled target fully covered by subsequent FOV deletes flattens to 0 points. Additionally line 348 then unlocks a mutex this thread does not own — harmless only because Phase 2.5 made it recursive (EPERM), but it documents that the original author assumed the lock was held here.

**Evidence.**

```
Line 287 'pthread_mutex_unlock(&working_flag_mutex);' -> line 291 'if (int(Rebuild_PCL_Storage.size()) > 0)' -> line 295 'pthread_mutex_lock(&working_flag_mutex);' is conditional, but lines 315-346 (swap block under only search_rw unique), 347 'Rebuild_Ptr = nullptr;', 348 'pthread_mutex_unlock(&working_flag_mutex);', 351 'delete_tree_nodes(&old_root_node);' run unconditionally. Mutators rely on working_flag only: Add_Points line 494 'ScopedPthreadLock whole_op_lock(&working_flag_mutex);' with the comment 'They must NOT run under search_rw_mutex_' (lines 560-566).
```

**Reference.** Hazard 35 in /home/user/RESPLE/CLAUDE.md ('recursive whole-op working_flag_mutex: ... excluding the rebuild thread's father_ptr read + swap ancestor Update-walk'); same conditional-lock structure exists in refs/upstream_ikd_Tree.cpp and refs/ikd_Tree_upstream.cpp, but upstream never claimed mutator-vs-rebuild exclusion via this mutex — the local fix does.

**Suggested fix.** Re-acquire working_flag_mutex unconditionally before the '/* Replace to original tree*/' block (move the lock out of the size()>0 branch, or add a matching pthread_mutex_lock in an else branch), so the swap, Rebuild_Ptr clear, and unlock at 348 are always performed while owning the mutex.

### 9. [MEDIUM] Worker heartbeat reads callback-mutated pc_buff.size() without mtx_pc — hazard-36-class data race left behind by the Phase 2.6 fix

- **Location:** `resple/src/RESPLE.cpp:559`
- **Found by:** concurrency-fresh  ·  **Finder confidence:** 0.85
- **Verification:** _pending_

**Description.** The ~2 s heartbeat block in processData() (worker thread) iterates lidars_data and reads d.pc_buff.size() with NO lock, while every sensor callback concurrently mutates the same deque under lidar_data.mtx_pc via pushScanBounded (RESPLE.cpp:1505-1514, push_back/pop_front). This is exactly the deque-internals data race class fixed as hazard 36 (Phase 2.6, commit 80d9980 'two data-path races found by live TSan sweep') — that commit fixed the drain loop's unlocked empty() check in the SAME function (RESPLE.cpp:590-596) and the diagnostic cache block ten lines above the heartbeat correctly takes mtx_pc for the identical read (RESPLE.cpp:543-546, comment: 'pc_buff is callback-written -> read under mtx_pc'), but the heartbeat read (added earlier, commit 181800d) was missed. The block's own comment ('Diagnostic only — read-only, no lock-order interactions') is wrong: a lock-free size() read racing a locked push_back is still UB. This makes the hazard-36 fix incomplete, which the task brief explicitly flags as a valid finding.

**Evidence.**

```
RESPLE.cpp:557-561 (worker thread, no lock):
  size_t pc_buff_total = 0, pt_buff_total = 0;
  for (auto& [n, d] : lidars_data) {
      pc_buff_total += d.pc_buff.size();   // <-- callback-written deque, no mtx_pc
      pt_buff_total += d.pt_buff.size();   // (pt_buff is worker-owned, OK)
  }
versus the correctly-locked sibling read at RESPLE.cpp:541-547:
  for (auto& [diag_name, diag_data] : lidars_data) {
      std::lock_guard<std::mutex> lk(diag_data.mtx_pc);
      lidar_total += static_cast<int64_t>(diag_data.pc_buff.size());
  }
and the concurrent writer RESPLE.cpp:1505-1513 (sensor callback):
  std::lock_guard<std::mutex> lock(ld.mtx_pc); ... ld.pc_buff.pop_front(); ... ld.pc_buff.push_back(...);
Sensor callbacks run on the executor's sensor_cb_group thread; the heartbeat runs on the dedicated processing_thread_, so the accesses are genuinely concurrent in the default LIO config, every 2 s. Consequence is a torn/garbage diagnostic count (libstdc++ deque::size() derives from two non-atomic iterators), formally UB, and a guaranteed TSan hit that would re-poison the now-clean TSan gate.
```

**Reference.** CLAUDE.md hazard 36 ('processData lidar-buffer drain checked t_buff.empty() OUTSIDE mtx_pc, racing the sensor callback's locked push_back on the deque internals — fixed Phase 2.6'); fix commit 80d9980 touched only the drain loop, not this read; correct pattern at RESPLE.cpp:541-547.

**Suggested fix.** Inside the heartbeat loop take the per-lidar lock (std::lock_guard<std::mutex> lk(d.mtx_pc);) around the pc_buff.size() read — or simply reuse cached_lidar_buf_ (already refreshed under the lock a few lines above) for pc_buff_total and keep only the worker-owned pt_buff read lock-free.

### 10. [MEDIUM] transformImu mixes units when acc_ratio=true: centripetal lever-arm correction gets scaled by 9.81x

- **Location:** `resple/src/RESPLE.cpp:2142`
- **Found by:** imu-path  ·  **Finder confidence:** 0.75
- **Verification:** _pending_

**Description.** transformImu() runs in getImuCallback on the RAW IMU message, before the acc_ratio g-to-m/s^2 scaling that the worker applies when draining the staging buffer. It subtracts the centripetal term omega x (omega x r), which is intrinsically in m/s^2 (rad/s and meters), from an accelerometer vector that is still in g-units when acc_ratio=true. The worker then multiplies the whole transformed vector by 9.81 (RESPLE.cpp:643 'if (acc_ratio) acc *= 9.81;'), so the fused accel becomes 9.81*R*a - 9.81*omega x (omega x r) instead of 9.81*R*a - omega x (omega x r): the lever-arm correction is inflated 9.81x. Every acc_ratio=true LIO config (config_helmdyn01/tudorun01/rcampus/rug_bb/demonstrator/heap_testsite, i.e. Livox built-in IMUs) hits this whenever a TF from the IMU frame to base_link exists (updateImuTransform succeeds). The production Ouster config (acc_ratio=false) is unaffected.

**Evidence.**

```
RESPLE.cpp:2142-2143: 'Eigen::Vector3d lin_accel_transformed = transform_eigen.rotation() * lin_accel - ang_vel_transformed.cross(ang_vel_transformed.cross(transform_eigen.translation()));' operates on lin_accel taken directly from imu_raw->linear_acceleration (g-units when acc_ratio). The scaling happens only later in the worker drain, RESPLE.cpp:643: 'if (acc_ratio) acc *= 9.81;' applied to the already-transformed sample pushed at line 2228. The health monitor at line 2185 ('const double a_scale = acc_ratio ? 9.81 : 1.0;') shows the code is aware raw samples are in g at callback time, yet transformImu ignores it. For omega=2 rad/s and r=0.05 m the injected error is ~8.81*0.2 = 1.8 m/s^2 during turns.
```

**Reference.** Rigid-body specific-force transfer f_B = R*f_I - alpha x r - omega x (omega x r) (all terms SI); upstream_RESPLE.cpp:113-126 has no transformImu (workspace-local addition, A4-fixed in commit 0b9e5f2) and applies acc_ratio scaling at the same drain point, so upstream never mixed units.

**Suggested fix.** Inside transformImu, scale lin_accel by 9.81 first when acc_ratio is set and mark the message as already-SI (or move the acc_ratio scaling from the worker drain into getImuCallback before transformImu, for both pre- and post-init paths, and delete the 'acc *= 9.81' in the drain).

### 11. [MEDIUM] last_pose_pub_time_ns_ never reset on lifecycle cleanup — pose/odom/TF silently suppressed after a re-cycle with non-advancing timestamps

- **Location:** `resple/src/RESPLE.cpp:2877`
- **Found by:** node-logic, concurrency-fresh  ·  **Finder confidence:** 0.72
- **Verification:** _pending_

**Description.** publishPoseAndTf() dedups by `if (pose_time_ns <= last_pose_pub_time_ns_) return;`. The member (line 2869) is initialized once and is the ONLY publish-path state not reset in on_cleanup: lines 386-407 reset knot_rot_checked_, if_init_filter/if_init_map, all one-shot log flags, etc., showing the code explicitly supports deactivate→cleanup→configure→activate re-cycles (hazards 17/18 were fixed for exactly this path). After a re-cycle where the new run's sensor timestamps are not strictly greater than the previous run's last published spline time — the standard bag-replay workflow this repo's CLAUDE.md prescribes ('replaying recorded production bags'), a looped bag, or a sim-time reset — every pose_time_ns is <= the stale last_pose_pub_time_ns_, so pub_pose, pub_pose_cov, pub_odom and the odom→base_link TF broadcast are all skipped for the entire overlap. The node looks alive (est_window, diagnostics, current_scan all publish) but produces no odometry.

**Evidence.**

```
Line 2869: `int64_t last_pose_pub_time_ns_ = std::numeric_limits<int64_t>::min();` — a member, not a processData local (unlike max_spl_knots / t_last_map_upd at lines 514-515 which are re-created per activation). Lines 2877-2878: `if (pose_time_ns <= last_pose_pub_time_ns_) return; last_pose_pub_time_ns_ = pose_time_ns;`. on_cleanup (lines 386-407) resets every other cross-cycle member but not this one; grep confirms only 3 occurrences (2869, 2877, 2878) — no reset site.
```

**Reference.** on_cleanup reset list at RESPLE.cpp:386-407 (the established re-cycle-reset pattern this member was omitted from); CLAUDE.md hazards 17/18 establish lifecycle re-cycle as a supported path.

**Suggested fix.** Reset `last_pose_pub_time_ns_ = std::numeric_limits<int64_t>::min();` in on_cleanup (or at the top of processData / on_activate).

### 12. [MEDIUM] fitPlane solves the plane fit via float32 normal equations (ATA), squaring the condition number — normals degrade/reject with distance from the odom origin (upstream solves A directly)

- **Location:** `resple/include/utils/geometry_core.h:67`
- **Found by:** association  ·  **Finder confidence:** 0.7
- **Verification:** _pending_

**Description.** esti_plane (Association.h:142 calls it with T=float, pabcd is Vector4f) now delegates to resple::geom::fitPlane, which forms the 3x3 normal-equation matrix ATA = sum(p p^T) in float32 and solves it with colPivHouseholderQr. The vendor-point upstream and FAST-LIO both solve the 5x3 system A n = -1 directly with colPivHouseholderQr on A. Normal equations square the condition number; the rows are UNCENTERED world-frame coordinates, so cond(A) ~ |position|/patch_extent grows with distance from the odom origin, and cond(ATA) grows with its square. In float32 (eps ~ 1.2e-7) this destroys the solve for trajectories a few hundred meters from the start. Monte-Carlo verification (5-point patches, 0.5 m extent, 1 cm noise, float32 both ways): at 300 m median normal error 3.05 deg (QR: 1.45 deg), p95 20.5 deg; at 600 m 17% of fits fail the 0.1 m residual check outright and median error is 11.5 deg; at 1 km 61% rejected, 27.6 deg median. Accepted-but-wrong normals directly corrupt zp and the H row in prepLiDAR; rejected fits starve the IEKF of correspondences exactly at long range. cube_len=1000 in the shipping config indicates km-scale operation is in scope. The comment claiming behaviour is "bit-for-bit identical to the previous inline normal-equation fit" is true only relative to the intermediate commit 0600aad, which itself introduced this regression away from the vendor-point QR.

**Evidence.**

```
geometry_core.h:61-70: `Eigen::Matrix<T, 3, 3> ATA = ...Zero(); ... ATA.noalias() += row * row.transpose(); ATb.noalias() -= row; ... Eigen::ColPivHouseholderQR<Eigen::Matrix<T, 3, 3>> qr(ATA); ... normvec = qr.solve(ATb);` with T=float from common_utils.h:286 esti_plane<float> called at Association.h:142 (`Eigen::Vector4f pabcd`). Upstream vendor point (upstream_common_utils.h:216-231): `Eigen::Matrix<T, 5, 3> A; ... b *= -1.0f; ... Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);` — QR on A itself, condition number NOT squared. Simulation (float32, 500 trials/distance): dist=300: reject QR=0.000 NE=0.000, angerr(med) QR=1.454deg NE=3.052deg, p95 NE=20.48deg; dist=600: reject NE=0.170, med NE=11.476deg; dist=1000: reject NE=0.612, med NE=27.646deg.
```

**Reference.** upstream_common_utils.h:216-231 (vendor point 3d0e78c); FAST-LIO common_lib.h esti_plane (solves A x = -1 via matA0.colPivHouseholderQr().solve(matB0), no normal equations); standard LS numerics: cond(A^T A) = cond(A)^2

**Suggested fix.** Solve the k x 3 system directly (Eigen::Matrix<T, Eigen::Dynamic, 3, 0, 8, 3> A; A.colPivHouseholderQr().solve(b)) as upstream did, or keep normal equations but subtract the patch centroid before forming ATA (fit the normal on centered coordinates, then d = -n.dot(centroid)); either restores float32 accuracy independent of distance from origin.

### 13. [MEDIUM] processScan's deskew-lag gate and drop-old test use points.back() of a VoxelGrid-filtered cloud as 'scan end' — the filter output is leaf-ordered, not time-ordered

- **Location:** `resple/src/Mapping.cpp:193`
- **Found by:** mapping-node  ·  **Finder confidence:** 0.7
- **Verification:** _pending_

**Description.** Every sensor callback buffers pc_last_ds, the output of pcl::VoxelGrid (ds_filter_each_scan.filter(*pc_last_ds)). VoxelGrid emits one centroid per occupied voxel in ascending leaf-index (spatial) order and, with default downsample_all_data, averages the intensity field (which here encodes per-point relative time in ms). So `front.points.back().intensity` is the mean time of an arbitrary spatial corner voxel — a value anywhere in [0, scan_period], not the scan end. The §6.3 deskew-lag mechanism (be99a7e/fb069df) is built on this: the gate `t_end_ns > spl->maxTimeNs() - lag_ns` is supposed to guarantee the spline edge is deskew_lag_knots (default 8, 80 ms at knot_hz 100) past the scan's END, but with a bogus t_end the true tail can extend up to a full scan period (100 ms for the OS1 at 10 Hz) past the assumed end. Consequences: (a) the lag guarantee is void — tail points are still deskewed with under-converged trailing-edge knots, the exact smear §6.3 set out to fix; (b) tail points whose true t_ns lands beyond spl->maxTimeNs() are silently discarded by transformCloud's window check (line 343), carving time-varying slices out of the published map; (c) the dropped-as-old test (t_end_ns < minTimeNs, line 196) can misclassify. Upstream has the same expression (refs/upstream_Mapping.cpp:47), but upstream only used it as a coarse in-window test; the local lag feature turned it into a correctness-bearing quantity.

**Evidence.**

```
Mapping.cpp:193-194: `const int64_t t_end_ns = front.header.stamp + int64_t(front.points.back().intensity * float(1e6));` operating on clouds produced by `ds_filter_each_scan.filter(*this->pc_last_ds)` (e.g. Mapping.cpp:444-453). Gate at Mapping.cpp:201: `if (t_end_ns > spl->maxTimeNs() - lag_ns)`; silent tail drop at Mapping.cpp:343: `if (t_ns >= spl->minTimeNs() && t_ns <= spl->maxTimeNs())`.
```

**Reference.** pcl::VoxelGrid<PointT>::applyFilter (pcl/filters/impl/voxel_grid.hpp): indices sorted by voxel idx, output emitted per unique leaf in that order, fields averaged when downsample_all_data_ (default true). Upstream parity: refs/upstream_Mapping.cpp:47 (same expression, pre-lag semantics).

**Suggested fix.** Record the true scan-end offset at ingest time (max over raw per-point times before the voxel filter — the callbacks already iterate every point) in a parallel deque alongside pc_L_buff, or compute t_end as max intensity over the filtered cloud in processScan; use that for both the lag gate and the drop-old test.

### 14. [MEDIUM] Points_deleted / Multithread_Points_deleted grow without bound: acquire_removed_points is never called and Delete_Storage_Disabled is write-only

- **Location:** `resple/include/ikd-Tree/ikd_Tree.cpp:1661`
- **Found by:** ikdtree  ·  **Finder confidence:** 0.7
- **Verification:** _pending_

**Description.** flatten() records every lazily-deleted, non-downsample point into member vectors: DELETE_POINTS_REC -> Points_deleted (line 1655, from every inline Rebuild() at line 854) and MULTI_THREAD_REC -> Multithread_Points_deleted (line 1661, from every background rebuild at line 284). The only code that ever clears these vectors is acquire_removed_points (lines 754-769), which has no caller anywhere in the RESPLE workspace (grep over resple/src and resple/include: zero call sites). Delete_Storage_Disabled is set in the destructor (line 42) but never read, so it gates nothing. In the default LIO config, lasermapFovSegment's Delete_Point_Boxes (RESPLE.cpp:3395/3444) lazily deletes every map point that scrolls out of the local cube with is_downsample=false, so point_downsample_deleted stays false and each such point is appended to these vectors when a rebuild later sweeps its subtree. On a long mission the vectors accumulate essentially every point ever dropped from the local map (tens of bytes each, millions of points) — a monotonic memory leak in a node that is expected to run for hours. This is inherited from the vendor point but is not in the 38-hazard list, and the Phase 5 LeakSanitizer gate cannot see it (the memory is still reachable).

**Evidence.**

```
flatten lines 1652-1662: 'case DELETE_POINTS_REC: if (root->point_deleted && !root->point_downsample_deleted) { Points_deleted.push_back(root->point); } ... case MULTI_THREAD_REC: ... Multithread_Points_deleted.push_back(root->point);'. acquire_removed_points (754) is the sole clear; 'grep -rn acquire_removed_points resple --include=*.cpp,*.h' returns only the ikd-Tree definition. 'Delete_Storage_Disabled' appears exactly once outside its declaration, as a write (line 42).
```

**Reference.** refs/upstream_ikd_Tree.cpp has the identical recording logic (also never gated), and FAST-LIO-style consumers are expected to drain via acquire_removed_points; RESPLE (both upstream and local Mapping/RESPLE nodes) never does.

**Suggested fix.** Since RESPLE never consumes removed points, stop recording them: pass NOT_RECORD from Rebuild()'s inline flatten and multi_thread_rebuild's flatten (or honor Delete_Storage_Disabled / add a member flag set at construction), keeping the storage_type plumbing for callers that need it.

### 15. [MEDIUM] q_lb/t_lb documented as 'LiDAR->body' but the code convention is body(IMU)->lidar — doc-following users supply the inverted extrinsic

- **Location:** `doc/PARAMETERS.md:93`
- **Found by:** config-units  ·  **Finder confidence:** 0.7
- **Verification:** _pending_

**Description.** PARAMETERS.md line 93 documents '<name>/q_lb, <name>/t_lb | LiDAR->body extrinsics', and config_pointcloud2.yaml:36 says 'lidar->body extrinsic rotation'. The code's actual convention is the opposite: Mapping.cpp:281-282 builds 'Eigen::Affine3d imu_to_lidar = Translation3d(lidar.t_lb) * lidar.q_lb' (i.e. (q_lb,t_lb) maps IMU/body coords into the lidar frame), and the upstream dataset values confirm it — config_helmdyn01.yaml t_lb [0.011, 0.02329, -0.04412] is exactly the Livox Mid-360 datasheet position of the built-in IMU expressed in the LIDAR/point-cloud frame. common_utils.h:336-337 accordingly inverts it (q_bl = q_lb.inverse(); t_bl = q_lb.inverse()*(-t_lb)) to get lidar->body. A user of the generic PointCloud2 template who follows the written 'lidar->body' direction with a non-trivially-rotated sensor will feed the estimator the inverse extrinsic; for a 90-deg mount that is a 180-deg total error relative to intent. (For the identity extrinsics in the production config this is latent.)

**Evidence.**

```
PARAMETERS.md:93 '| `<name>/q_lb`, `<name>/t_lb` | — | LiDAR->body extrinsics (quaternion `w,x,y,z`; translation m) |' vs Mapping.cpp:281-283 'Eigen::Translation3d translation(lidar.t_lb); Eigen::Affine3d imu_to_lidar = translation * lidar.q_lb; imu_to_baselink_ = lidar_to_baselink_ * imu_to_lidar;' and common_utils.h:336-337 'q_bl = q_lb.inverse(); t_bl = q_lb.inverse() * (- t_lb);'. config_helmdyn01.yaml:28 t_lb [0.011, 0.02329, -0.04412] = Mid-360 IMU offset (11.0, 23.29, -44.12) mm in the point-cloud frame.
```

**Reference.** Livox Mid-360 user manual: IMU located at (11.0, 23.29, -44.12) mm in the point-cloud coordinate frame; Mapping.cpp:282 in-repo naming imu_to_lidar.

**Suggested fix.** Correct PARAMETERS.md and the config template comments to 'body(IMU)->lidar extrinsics: t_lb = position of the body/IMU origin expressed in the lidar frame' (or rename the parameters when finding 1 is resolved).

### 16. [MEDIUM] updateKnots appends across index gaps at the wrong deque position, permanently shifting the replica spline's time axis; Mapping-node restart mid-run breaks map/odom irrecoverably

- **Location:** `resple/include/SplineState.h:239`
- **Found by:** mapping-node  ·  **Finder confidence:** 0.65
- **Verification:** _pending_

**Description.** When a window arrives with target = i + other->start_i - num_knots_pruned_ > num_knot (an est_window gap), updateKnots logs and then addOneStateKnot()s anyway, placing absolute knot `target` at local index `num_knot`. Since knot time = start_t_ns + local_index*dt, every knot from then on carries a timestamp gap*dt EARLIER than its true time. The num_knot==0 branch (lines 221-227) declares this "startup origin; expected after (re)start" — but the misalignment is identical: windows published between RESPLE's start_time and the Mapping worker consuming start_pending_ are dropped by design (getEstCallback returns while !if_init_succeed, Mapping.cpp:1323; the worker sleeps up to 100 ms before consuming the staged start, Mapping.cpp:1183-1186), so the first applied window begins at start_idx > 0 and the whole replica runs a few knots (tens of ms) early — a permanent stamp-vs-pose offset in traj_path, /odometry, the map→odom TF pairing, and the deskew. The severe case: a Mapping-only restart mid-run. start_time is transient_local, so startCallBack re-stages with the ORIGINAL bag start (Mapping.cpp:1109 init(1, 0, start_bag_time_pending_, 0)), and the first applied window has start_idx ≈ current totalKnots — the replica's maxTimeNs() is then behind wall time by gap*dt forever (each window only appends ~1 knot), so processScan's gate `t_end_ns > maxTimeNs() - lag_ns` never releases another scan (map silently freezes) and path/odom interpolate poses at times shifted by the entire pre-restart duration. The gap COUNTER (ffac28c) detects but never corrects; the message already carries dt/start_idx/start_t, so resynchronization is trivially possible.

**Evidence.**

```
SplineState.h:219-239: `if (target > num_knot) { update_gap_events_++; if (num_knot == 0) { std::cerr << "...startup origin; expected after (re)start" ... } else ... "knot times misaligned from here" } addOneStateKnot(other->t_knots[i], other->ort_delta[i]);` — the append happens at local index num_knot regardless of target, so getKnotTimeNs(local) understates the true knot time by (target-num_knot)*dt for all subsequent knots. Mapping.cpp:1109: `spline_active_.init(1, 0, start_bag_time_pending_.load(...), 0);` re-anchors start_t_ns to the ORIGINAL bag start on every (re)start.
```

**Reference.** est_window protocol contract documented at SplineState.h:663-681 (absolute totalKnots() index space, start_t = time of knot total-5) and getSplineMsg contiguity clamp SplineState.h:667-668 (start advances ≤1 per publish — so any dropped window creates a real gap); commits ffac28c/a84b7f4 fixed only the queue-overwrite drift, not the gap time-base.

**Suggested fix.** In updateKnots, when target > num_knot, advance the time axis instead of compressing it: start_t_ns += (target - num_knot) * dt_ns (equivalently treat the skipped knots as pruned: num_knots_pruned_ += target - num_knot) before appending, so appended knots land at their true absolute times. For the num_knot==0 case this also makes a Mapping-only restart converge to the live edge.

### 17. [MEDIUM] Q_block_new written to the bias block via bottomRightCorner on the 30x30 Q: newest RCP gets zero process noise, ba/bg get a large pose-scaled random walk (inherited upstream bug)

- **Location:** `resple/src/RESPLE.cpp:2007`
- **Found by:** imu-path  ·  **Finder confidence:** 0.6
- **Verification:** _pending_

**Description.** initFilter builds the process-noise matrix Q as 30x30 and writes Q_block_old to blocks (0,0),(6,6),(12,12) and Q_block_new to bottomRightCorner<6,6>() -- which on a 30x30 matrix is rows/cols 24-29, the accel/gyro BIAS block, not rows/cols 18-23 where the newest RCP lives (proven by a_mat: Estimator.h:149-151 sets the new-knot extrapolation rows at block row 18). Consequences: (1) in LIO the newest control point receives zero additive process noise while the IMU biases receive a random walk of cov_RCP_pos_new*cov_sys_pos (position units!) on ba and cov_RCP_ort_new*cov_sys_ort on bg at every knot step (~100 Hz), which is not any documented bias model and lets the estimated biases wander with pose-tuned magnitudes; (2) in LO, setState receives Q.topLeftCorner<24,24>() so Q_block_new vanishes entirely -- the cov_RCP_pos_new/cov_RCP_ort_new parameters that every shipped LO config sets to 1.0 are dead. The 'old'/'new' param naming and the LO dead-parameter effect show the intended target was block (18,18) (bottomRightCorner of the original 24x24 layout).

**Evidence.**

```
RESPLE.cpp:1997 'Eigen::Matrix<double, 30, 30> Q = ...Zero();' then 2004-2007: 'Q.topLeftCorner<6, 6>() = Q_block_old; Q.block<6, 6>(6, 6) = Q_block_old; Q.block<6, 6>(12, 12) = Q_block_old; Q.bottomRightCorner<6, 6>() = Q_block_new;'. bottomRightCorner<6,6> of a 30x30 is block (24,24) = [ba; bg] (Estimator.h BA_OFFSET=24, BG_OFFSET=27). Block (18,18) -- the newest knot per a_mat.block(18,0,3,3)=-I, a_mat.block(18,12,3,3)=2I, a_mat.block(21,9,3,3)=I in Estimator.h:149-151 -- stays zero. LO path (line 2009) takes Q.topLeftCorner<24,24>(), discarding Q_block_new completely.
```

**Reference.** Inherited verbatim from upstream_RESPLE.cpp:303-313 (same bottomRightCorner on 30x30); param semantics cov_RCP_{pos,ort}_new documented as newest-RCP noise in doc/PARAMETERS.md and set in all shipped configs including LO ones (config_helmdyn01.yaml) where it currently has no effect.

**Suggested fix.** Replace 'Q.bottomRightCorner<6, 6>() = Q_block_new;' with 'Q.block<6, 6>(18, 18) = Q_block_new;' and give the bias block its own (small or zero) random-walk parameter, e.g. Q.block<3,3>(24,24)/Q.block<3,3>(27,27) from new cov_ba_rw/cov_bg_rw params. Re-tune/validate on bags since this changes effective process noise in both modes.

### 18. [MEDIUM] Gravity alignment and all pre-init IMU samples bypass the base_link extrinsic: initial orientation is wrong by the IMU mounting rotation and the tilt can be permanently locked in

- **Location:** `resple/src/RESPLE.cpp:2220`
- **Found by:** imu-path  ·  **Finder confidence:** 0.6
- **Verification:** _pending_

**Description.** Pre-init, getImuCallback buffers RAW IMU samples ('if (!init_done) { imu_int_buff.push_back(imu_msg); return; }') on the justification that gravity direction is frame-independent -- which is false when R_base_imu is not identity: the accelerometer measures gravity in the IMU frame, and g2R alignment of that vector yields q_W_IMU, but initFilter (line 3163) installs it as the spline's base_link orientation q_W_B. The initial roll/pitch is therefore wrong by the base_link<-imu mounting rotation, the first map seed is tilted by that amount, and the first post-init IMU measurements consumed by the IEKF are also still raw IMU-frame samples (staged pre-init, drained after the flag flip). In LIO the filter can only slew the tilt out via accel residuals, but updateLiDARInertial gates any per-axis accel residual above 10 m/s^2 (Estimator.h:847) -- a mounting rotation of ~60 degrees or more (e.g. a 90-degree or inverted mount) produces residuals ~sqrt(2)*9.81 that are permanently zeroed, so the tilt is never corrected and the map stays misaligned with gravity. In LO mode there is no correction mechanism at all: the 'ensures the spline starts gravity-aligned (z = up)' goal of the always-use-IMU init (line 3058) silently fails by the mounting rotation.

**Evidence.**

```
RESPLE.cpp:2217-2222: '// Pre-init: accept raw IMU for gravity alignment. Gravity direction is\n// frame-independent for roll/pitch -- the accelerometer measures g\n// regardless of the sensor's mounting frame.\nif (!init_done) { imu_int_buff.push_back(imu_msg); return; }' -- the transformImu(imu_msg, imu_to_baselink_) call at line 2227 runs only post-init. initialization() line 3147-3152 computes q_WI from those raw samples and passes it to initFilter as the base-frame spline orientation. Estimator.h:846-851 zeroes accel rows with |residual| > 10.0 m/s^2, blocking convergence for large mount rotations.
```

**Reference.** Standard frame transformation: a_base = R_base_imu * a_imu; g2R (common_utils.h:206, VINS-Mono Utility::g2R) aligns the argument vector with +z, so feeding IMU-frame accel yields the IMU-frame attitude, not base_link's. The production Ouster extrinsic is ~identity rotation (config_06042026.yaml header: os_sensor->os_imu t=[-0.0024,-0.0097,0.0075], identity), so this is latent there but live for any rotated mount.

**Suggested fix.** Rotate pre-init samples into base_link when the TF is available (apply at least the rotation part of transformImu before buffering), or rotate gravity_mean by R_base_imu in initialization() before g2R; log a WARN when gravity alignment ran without the extrinsic and the transform later turns out to have a non-identity rotation.

### 19. [MEDIUM] Data race on non-atomic Rebuild_Ptr read in searcher-path Push_Down (double-load can null-deref)

- **Location:** `resple/include/ikd-Tree/ikd_Tree.cpp:1361`
- **Found by:** ikdtree  ·  **Finder confidence:** 0.6
- **Verification:** _pending_

**Description.** Push_Down decides its rebuild branch with 'if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != child)' (lines 1361 and 1415). When called from the search path (Search/Search_by_range/Search_by_radius/flatten via Nearest_Search/Box_Search/Radius_Search/flatten_safe), the thread holds only search_rw_mutex_ shared — not working_flag_mutex. The rebuild thread writes 'Rebuild_Ptr = nullptr' at line 347 AFTER its search_rw unique block closes at 346 (holding only working_flag_mutex), and mutators write 'Rebuild_Ptr = root' in Rebuild() at line 844 under working_flag_mutex only. Rebuild_Ptr is a plain KD_TREE_NODE** (ikd_Tree.h:322), so this is a C++ data race (UB, TSan-reportable), and because the two loads in the short-circuit expression are unsynchronized the compiler may legally reload Rebuild_Ptr between the null check and the '*Rebuild_Ptr' dereference — a concurrent 'Rebuild_Ptr = nullptr' then yields a null-pointer dereference in the IEKF's OpenMP Nearest_Search or in the map-publish flatten_safe. The Phase 2.4 hardening made the KD_TREE_NODE flag fields atomic specifically to make search-side lock-free reads defined behaviour, but left this shared pointer non-atomic with the same lock-free readers. (tree_range/validnum/root_alpha at lines 115/155/180 have the same racy pattern but are currently uncalled.)

**Evidence.**

```
Line 1361 'if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != child)' executed with only search_rw shared held by the caller (Nearest_Search line 452); writer at line 347 'Rebuild_Ptr = nullptr;' sits outside the unique_lock block that ends at line 346, so shared-lock holders are not excluded; writer at line 844 'Rebuild_Ptr = root;' in Rebuild() runs under working_flag_mutex which searchers do not hold. ikd_Tree.h:322 declares 'KD_TREE_NODE **Rebuild_Ptr = nullptr;' (non-atomic).
```

**Reference.** Same racy pattern exists at the vendor point (refs/upstream_ikd_Tree.cpp Push_Down), but the local Phase 2.4/2.5 work (CLAUDE.md hazards 33/35, 'TSan ... suppressions file emptied') claims the search-vs-rebuild races are closed; this read was missed. Contrast with the local size() fix comment at lines 99-108, which removed exactly this kind of lock-free Rebuild_Ptr/Root_Node fast-path.

**Suggested fix.** Make Rebuild_Ptr std::atomic<KD_TREE_NODE**> (load once into a local before the null check and dereference), or move the 'Rebuild_Ptr = nullptr' write at line 347 inside the search_rw unique block and have searchers snapshot it under their shared lock; the same treatment fixes rebuild_flag being cleared at line 349 outside working_flag_mutex.

### 20. [MEDIUM] Generic PointCloud2 time_unit 'auto' misclassifies float64 nanosecond timestamp fields (livox_ros_driver2 convention): after the first scan, every subsequent scan is dropped by the monotonic gate

- **Location:** `resple/include/utils/point_cloud_adapter.h:212`
- **Found by:** config-units  ·  **Finder confidence:** 0.6
- **Verification:** _pending_

**Description.** timeUnitToMs Auto assumes 'Float fields are conventionally seconds; integer fields nanoseconds' (line 212), and the AdapterConfig comment (lines 143-145) claims absolute-epoch 'Livox int64 ns' is handled. But the Livox ROS2 driver's PointCloud2 output stores per-point absolute time as a FLOAT64 field named 'timestamp' holding NANOSECONDS — exactly the convention this repo's own livox_mid360_boxi::Point uses (common_utils.h:95 'double timestamp', consumed as ns at RESPLE.cpp:2841 'rclcpp::Time(static_cast<int64_t>(...timestamp))'). Through the generic path with the default time_unit: auto, FLOAT64 -> scale 1.0e3 (seconds->ms), so after min-normalization the per-point offsets are ns*1e3 'ms' = 1e9x too large. The first scan is accepted (offsets pass >=0 and the last_t_ns gate), but genericLidarCallback then stores last_t_ns = time_begin + max_ofs_ns where max_ofs_ns = ms2ns(offset) ~ 1e17 ns (~3 years in the future) (RESPLE.cpp:2538-2551). Every later scan fails 'ofs + time_begin > last_t_ns' for all points and is dropped entirely — the estimator consumes exactly one scan and then starves, silently.

**Evidence.**

```
point_cloud_adapter.h:212 'return (datatype == FLOAT32 || datatype == FLOAT64) ? 1.0e3 : 1.0e-6;' + lines 143-145 comment '...an absolute-epoch time field (Hesai double seconds, Livox int64 ns)...'. RESPLE.cpp:2538-2551: 'int64_t ofs = CommonUtils::ms2ns(pt.intensity); if (ofs + time_begin > last_t_ns) { max_ofs_ns = ...; }' ... 'lidar_buffs.last_t_ns.store(time_begin + max_ofs_ns);'. In-repo counter-example of the convention: common_utils.h:95 double timestamp treated as int64 ns at RESPLE.cpp:2841.
```

**Reference.** livox_ros_driver2 (hku-mars) PointCloud2 output: field 'timestamp' FLOAT64 in nanoseconds — same layout as this repo's livox_mid360_boxi::Point / Mid360Boxi callback (RESPLE.cpp:2841).

**Suggested fix.** In auto mode, after computing t_ms for the cloud, sanity-check the span (max-min): if the normalized span exceeds a plausible scan duration (e.g. >10 s), re-try the other unit interpretations and/or warn; or special-case FLOAT64 fields whose absolute magnitude is ~1e18 (epoch ns) as nanoseconds. Document 'time_unit: ns' as required for livox_ros_driver2 in config_pointcloud2.yaml.

### 21. [LOW] PARAMETERS.md documents cov_pose/cov_twist defaults as [0.1 x6]; the code default is {0.2, 0.2, 0.2, 0.1, 0.1, 0.1}

- **Location:** `doc/PARAMETERS.md:81`
- **Found by:** config-units  ·  **Finder confidence:** 0.9
- **Verification:** _pending_

**Description.** PARAMETERS.md line 81 states '| `cov_pose`, `cov_twist` | `[0.1 x6]` | Diagonals for the Mapping node's odometry covariance |'. The actual code defaults are {0.2, 0.2, 0.2, 0.1, 0.1, 0.1} in both nodes (RESPLE.cpp:1968 for cov_pose, Mapping.cpp:921 and :924 for cov_pose/cov_twist). A user omitting the parameters (the doc says 'Every parameter is optional — omitting it keeps the listed default') gets 2x the documented translation variances published downstream.

**Evidence.**

```
RESPLE.cpp:1968 'readParam<std::vector<double>>(..., "cov_pose", {0.2, 0.2, 0.2, 0.1, 0.1, 0.1})'; Mapping.cpp:921/924 same defaults for cov_pose/cov_twist; PARAMETERS.md:81 claims [0.1 x6]. The example YAMLs (config_ouster.yaml:16-17) set 0.1 x6, masking the drift.
```

**Reference.** RESPLE.cpp:1968, Mapping.cpp:921-924 vs doc/PARAMETERS.md:81

**Suggested fix.** Update PARAMETERS.md to the real defaults (or change the code defaults to the documented 0.1 x6 to match the example configs).

### 22. [LOW] NIS dof counts zeroed (gate-rejected / outlier-clamped) measurement rows, biasing the windowed NIS/dof consistency statistic low

- **Location:** `resple/include/Estimator.h:877`
- **Found by:** spline-math  ·  **Finder confidence:** 0.7
- **Verification:** _pending_

**Description.** update() sets `last_nis_dof_ = num_pts` where num_pts = innov.rows(). But rows in H_buf_/innv_buf_ are pre-zeroed and remain zero for (a) LiDAR points failing the accept gate `abs(zp) < pt_thresh || lid_cov < var_pt*cov_thresh` (Estimator.h:760-767, 817-825 — the row slot is still consumed via idx_offset++), and (b) IMU axes clamped by the 10 m/s^2 / 5 rad/s outlier check (lines 846-857, innovation and H row zeroed). A zero row contributes exactly 0 to NIS = nu^T S^-1 nu but still counts 1 toward dof, so for a consistent filter E[NIS/dof] = accepted_rows/total_rows < 1 rather than 1. The NisDivergenceDetector thresholds (warn_ratio=2.0, diverged_ratio=4.0 on the windowed mean of NIS/dof, filter_health.h:42-45, 72) are calibrated to 'consistent filter expects ~1.0', so the reject fraction directly dilutes detection sensitivity — and the dilution grows in degraded scenes (more rejects) where detection matters most. The published dmsg.nis_dof (RESPLE.cpp:1205/1209) is likewise inflated. The overconfident-divergence case (small lid_cov lets large-zp rows through) still triggers, so this degrades rather than disables the detector.

**Evidence.**

```
Estimator.h:874-877: `int num_pts = innov.rows(); ... last_nis_dof_ = num_pts;`. Estimator.h:760-773 (LO): rows failing `if (abs(pt_data.zp) < pt_thresh || lid_cov < pt_data.var_pt*cov_thresh)` leave `innv_buf_(idx_offset)`/`H_buf_.row(idx_offset)` at their setZero() values yet `idx_offset++` still consumes the row; same at 817-831 (LIO), and IMU clamp at 846-857 zeroes innov+H rows that stay in dim_meas. Zero rows contribute 0 to `innov.dot(Rinv_nu) - b.dot(llt_S.solve(b))` (line 912) but 1 each to dof.
```

**Reference.** Bar-Shalom, Estimation with Applications to Tracking and Navigation, ch. 5: NIS ~ chi-square with dof = dimension of the actual innovation vector; filter_health.h:11-13 ('A healthy, consistent filter has E[NIS] = dof')

**Suggested fix.** Count dof as the number of rows actually carrying a measurement: increment a counter when a LiDAR row passes the accept gate and add 6 minus the clamped-axis count per IMU sample, and pass that as last_nis_dof_ (equivalently, count nonzero H rows).

### 23. [LOW] MapSaving service callback lets pcl::IOException escape — save with unwritable pcd_save_path aborts the node via std::terminate

- **Location:** `resple/src/MapSaving.cpp:78`
- **Found by:** concurrency-fresh, mapping-node, recent-commits  ·  **Finder confidence:** 0.7
- **Verification:** _pending_

**Description.** savePCDCallback calls pcl::io::savePCDFileBinary(pcd_save_path, map_copy) with no try/catch. PCDWriter::writeBinary throws pcl::IOException when the output file cannot be opened (nonexistent directory, read-only filesystem — an ordinary parameter misconfiguration, since pcd_save_path is a free-form parameter defaulting to /tmp/global_map.pcd). rclcpp does not catch user-callback exceptions; the throw propagates out of rclcpp::spin() in main() (line 87, also unguarded) to std::terminate -> SIGABRT with no useful log. This directly violates the package's own hardening convention (hazards 14/24: every callback body wrapped in try/catch precisely because an escaping exception terminates the executor), so 'kept verbatim' does not exempt it: the whole point of vendoring it into this hardened package was crash discipline. globalMapCallback's pcl::fromROSMsg (line 65) is similarly unguarded against a malformed /global_map message.

**Evidence.**

```
MapSaving.cpp:70-81:
  void savePCDCallback(...) {
      pcl::PointCloud<pcl::PointXYZI> map_copy;
      { std::lock_guard<std::mutex> lock(mtx_map); map_copy = *accumulated_map; }
      pcl::io::savePCDFileBinary(pcd_save_path, map_copy);   // <-- can throw pcl::IOException
      ...
  }
PCL 1.14 (system header /usr/include/pcl-1.14/pcl/io/impl/pcd_io.hpp:129-133):
  int fd = io::raw_open (file_name.c_str (), ...);
  if (fd < 0) { throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during open!"); }
No try/catch anywhere in MapSaving.cpp (unlike every RESPLE/Mapping callback, e.g. RESPLE.cpp:2154/2233, Mapping.cpp guardedCallback:146-155).
```

**Reference.** PCL 1.14 pcl/io/impl/pcd_io.hpp:133 (throw on open failure); CLAUDE.md hazards 14 and 24 (callback exceptions -> std::terminate, the established fix pattern); MapSaving added in commit 61ae1b8.

**Suggested fix.** Wrap the savePCDFileBinary call (and ideally the fromROSMsg in globalMapCallback) in try { ... } catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "map save failed: %s", e.what()); } and check the int return of savePCDFileBinary for < 0.

### 24. [LOW] Pre-init imu_buff is unbounded and the gravity all-windows variance scan is O(buff_size x n_imu) per 50 ms cycle — CPU/memory grow without bound while init is blocked

- **Location:** `resple/src/RESPLE.cpp:3105`
- **Found by:** node-logic  ·  **Finder confidence:** 0.65
- **Verification:** _pending_

**Description.** While gravity alignment fails the variance check (robot never stationary, imu_init_max_variance too low), initialization() returns false at line 3132 WITHOUT trimming imu_buff — the only trim (line 3141, pop < start_t_ns) runs after the variance check passes. Meanwhile every worker cycle drains the (capped) imu_int_buff staging into the uncapped imu_buff (lines 632-648), and the best-window search (lines 3105-3121) rescans ALL buff_size-n_imu+1 windows, each summing n_imu=50 samples, i.e. ~50 x buff_size operations 20 times per second. At 100 Hz IMU the cost grows linearly forever (60 min stuck: ~360k samples, ~18M ops per 50 ms iteration, plus ~30 MB/h of ImuData); the §2.3 'bounded input buffers' fix (hazard 5) capped imu_int_buff (max_imu_staging) but not this downstream deque, so the unbounded-buffer hazard survives on the blocked-init path in both LO and LIO modes.

**Evidence.**

```
Line 3105-3121: `for (int w = 0; w < n_windows; ++w) { ... for (int i = 0; i < n_imu; ++i) sum += imu_buff.at(w + i).accel; ... }` with `n_windows = buff_size - n_imu + 1` (line 3101); failure exit at 3125-3133 returns before the cleanup loop at 3141 (`while (!imu_buff.empty() && imu_buff.front().time_ns < start_t_ns) imu_buff.pop_front();`), which is only reached when accel_variance <= imu_init_max_variance_. Drain at 632-648 appends every staged sample to imu_buff each cycle with no cap.
```

**Reference.** CLAUDE.md hazard 5 / HARDENING §2.3 claims bounded input buffers via max_scan_buffer + max_imu_staging; max_imu_staging (RESPLE.cpp:2211-2215) caps only imu_int_buff, not imu_buff — the fix is incomplete on the blocked-init path. Also a running-sum window scan would be O(buff_size) instead of O(buff_size*n_imu).

**Suggested fix.** Cap imu_buff during pre-init (e.g. keep the newest N seconds, N >= imu_init_num_samples/rate, drop-oldest with a counter), and/or compute window means/variances with prefix sums so the scan is O(buff_size).

### 25. [LOW] setIdles chains q_idle / q_knots[0] with an off-by-one idle-delta convention, inconsistent with the 'delta arriving at knot j-3' convention established by pruneFrontKnots and prepareInterpolation

- **Location:** `resple/include/SplineState.h:163`
- **Found by:** iekf, association, numerics  ·  **Finder confidence:** 0.6
- **Verification:** _pending_

**Description.** The Phase 3.1 pruning code defines the idle-slot convention explicitly (comment at SplineState.h:248-253 and the pruneFrontKnots slide at 274-285): q_idle[j]/t_idle[j] hold the pose of knot j-3 and ort_delta_idle[j] holds the delta ARRIVING at it (q_{j-3} = q_{j-4} * exp(ort_delta_idle[j])). prepareInterpolation/itpPose read the idles under exactly this convention (cps[i] = knots_idle[i+idx_l+1] with cp0 = q_idle[idx_r-1] as the pre-window quaternion), and the bit-identity prune tests confirm it. setIdles, however, chains with the delta of the slot being set, one index off: setIdles(1) computes q_idle[2] = q_idle[1] * exp(ort_delta_idle[1]) — under the arriving convention it should be exp(ort_delta_idle[2]) — and setIdles(2) overwrites q_knots[0] = q_idle[2] * exp(ort_delta_idle[2]) instead of exp(ort_delta[0]) (knot 0's own arriving delta). setIdles is the receiver-side reconstruction path: Mapping::getEstCallback (Mapping.cpp:1344-1349) rebuilds the window spline's idles from getSplineMsg's raw ort_delta_idle publication, and updateKnots then copies q_idle/ort_delta_idle wholesale into the active spline when `num_knots_pruned_ == 0 && num_knot <= num_knots_other` (SplineState.h:201-205). Today this is benign because that guard restricts the copy to receiver startup, when the sender is also fresh and all idle deltas are zero (both conventions coincide at zero); a pruned sender (nonzero published idles) meeting a fresh receiver only happens in the Mapping-restart-against-running-RESPLE scenario, which is already acknowledged-degraded (updateKnots gap log). But the moment that guard is relaxed, the handshake is fixed, or any new caller feeds setIdles nonzero deltas, the receiver's q_idle chain — and hence orientation interpolation over the first two knot intervals — is silently wrong by one knot's rotation increment. This is a semantic-drift trap created by the pruning feature giving previously-always-zero fields a live meaning.

**Evidence.**

```
SplineState.h:158-168: `if (idx == 2) { ... q_knots[0] = q_idle[2] * q_del; } else if (idx == 1) { q_idle[2] = q_idle[1] * q_del; } else if (idx == 0) { q_idle[0] = q_idle0; q_idle[1] = q_idle[0] * q_del; }` where q_del = exp(the ort_del being stored into ort_delta_idle[idx]). Contrast pruneFrontKnots (274-285): `const int64_t src = prune - 3 + j; t_idle_new[j] = t_knots[src]; ort_idle_new[j] = ort_delta[src]; q_idle_new[j] = q_knots[src];` — slot j gets knot src's ARRIVING delta together with knot src's own pose, i.e. q_idle[j] = q_idle[j-1] * exp(ort_delta_idle[j]), one slot later than setIdles' chaining. Interpolation requires the pruning convention: for idx_l=0, cp0=q_idle[0] and t_delta[0]=ort_delta_idle[1] (prepareInterpolation:790-791 with cp0 selection itpQuaternion:381-388), so exp(ort_delta_idle[1]) must map q_idle[0]→q_idle[1].
```

**Reference.** Sommer et al. 2020 (refs/sommer2020_spline.txt) cumulative form q(u) = q_{i-3} ⊗ ∏ exp(λ_j d_{i-3+j}) fixes which delta belongs to which base quaternion; upstream refs/upstream_SplineState.h:97-112 has the identical setIdles chaining, but upstream never publishes nonzero idle deltas (no pruning), so the ambiguity was unobservable there. Local pruning comment SplineState.h:248-253 documents the arriving convention that setIdles contradicts.

**Suggested fix.** Make setIdles consistent with the arriving convention: setIdles(0) should only store slot 0 (no chain); after all three slots are stored, chain q_idle[1] = q_idle[0]*exp(ort_delta_idle[1]), q_idle[2] = q_idle[1]*exp(ort_delta_idle[2]), and leave q_knots[0] to be re-chained from ort_delta[0] by setOneStateKnot/updateRCPs (or use exp(ort_delta[0]) explicitly). Alternatively document the message-idle fields as departing deltas and convert in getSplineMsg. Add a test where a pruned sender's window (nonzero idles) is applied to a fresh receiver and the first two intervals' orientation is compared.

### 26. [LOW] Single cached lidar_to_baselink_ / have_lidar_transform_ shared across all LiDARs: in multi-LiDAR (MLO/MLIO) with TF extrinsics every sensor's cloud is transformed with whichever frame resolved first

- **Location:** `resple/src/RESPLE.cpp:2062`
- **Found by:** association  ·  **Finder confidence:** 0.6
- **Verification:** _pending_

**Description.** updateLidarTransform(source_frame_id) caches one Eigen transform (lidar_to_baselink_) guarded by one boolean (have_lidar_transform_). After the first sensor frame resolves, every subsequent call — including from a different LiDAR's callback with a different source_frame_id — returns true without a lookup, and all callbacks use the same lidar_to_baselink_ in pcl::transformPointCloud. sensor_origin_body (used for the point-to-plane range gate `range_sensor > 81*pd2*pd2` and the near-range noise inflation) is likewise set for ALL configured lidars from that single translation (RESPLE.cpp:2081-2083). In the supported multi-LiDAR modes (MLO/MLIO, e.g. the two-sensor heap_testsite config) with TF-based extrinsics, the second sensor's points are transformed with the first sensor's mount transform and its range gate uses the wrong origin.

**Evidence.**

```
RESPLE.cpp:2060-2100: `bool updateLidarTransform(std::string source_frame_id) { if (!have_lidar_transform_) { ... lidar_to_baselink_ = tf2::transformToEigen(transform); have_lidar_transform_ = true; for (auto& [name, lcfg] : lidars) { lcfg.sensor_origin_body = lidar_to_baselink_.translation(); } ... } return true; }` — no per-sensor keying; second frame_id short-circuits on the flag. Failure: MLIO with lidar A at identity mount and lidar B rotated 90 deg, both published in their own TF frames: whichever callback fires first pins lidar_to_baselink_; lidar B's clouds are then ingested with lidar A's transform -> systematically misregistered correspondences from sensor B.
```

**Reference.** Upstream design (upstream_Association.h/upstream_RESPLE.cpp) keeps per-lidar extrinsics in each LidarConfig (q_bl/t_bl per sensor) precisely because MLO is a supported mode; the TF path replaced that with a single shared transform.

**Suggested fix.** Key the cache per source frame (e.g. std::unordered_map<std::string, Eigen::Affine3d> + per-frame have flag), pass the per-sensor transform to the callback's transformPointCloud, and set each LidarConfig's sensor_origin_body from its own frame's translation.

### 27. [LOW] Generic PointCloud2 ingestion ignores row_step: organized clouds with padded rows are misparsed

- **Location:** `resple/include/utils/point_cloud_ingest.h:90`
- **Found by:** config-units  ·  **Finder confidence:** 0.55
- **Verification:** _pending_

**Description.** ingestPointCloud2 computes num_points = msg.width * msg.height and convertCloud indexes the blob as data + i * point_step (point_cloud_adapter.h:255), never consulting msg.row_step. For an organized cloud (height > 1) whose driver pads rows (row_step > width * point_step), every point after the first row is read at the wrong offset, yielding garbage xyz/time that flow into the estimator (non-finite values are filtered at RESPLE.cpp:2535, but finite-garbage values are not). The size guard at point_cloud_adapter.h:236-237 (num_points * point_step > data_size) does not reject this case because data_size = height * row_step is larger. pcl::fromROSMsg (used by all other callbacks) handles row_step correctly, so only the generic path is affected.

**Evidence.**

```
point_cloud_ingest.h:90 'const uint32_t num_points = msg.width * msg.height;' then convertCloud(msg.data.data(), msg.data.size(), msg.point_step, num_points, ...) — no row_step parameter exists anywhere in point_cloud_adapter.h; indexing at line 255: 'const uint8_t* pp = data + static_cast<size_t>(i) * point_step;'.
```

**Reference.** sensor_msgs/PointCloud2 spec: row access must use row_step (data[v*row_step + u*point_step]); cf. pcl_conversions/pcl::fromROSMsg which honors row_step.

**Suggested fix.** Pass msg.row_step into convertCloud and index pp = data + v*row_step + u*point_step; or reject clouds where height > 1 && row_step != width*point_step with a throttled warning.

### 28. [LOW] Omitting ds_scan_voxel / nn_thresh (documented as optional) yields degenerate 0.0 defaults: zero voxel leaf size and a disabled point-to-plane acceptance threshold

- **Location:** `resple/src/RESPLE.cpp:1725`
- **Found by:** config-units  ·  **Finder confidence:** 0.55
- **Verification:** _pending_

**Description.** PARAMETERS.md asserts 'Every parameter is optional — omitting it keeps the listed default, and the defaults reproduce the estimator's historical behaviour', but ds_scan_voxel (RESPLE.cpp:1717) and nn_thresh (RESPLE.cpp:1725) default to 0.0. With nn_thresh=0, the IEKF acceptance test 'abs(pt_data.zp) < pt_thresh || lid_cov < pt_data.var_pt*cov_thresh' (Estimator.h:760, 817) loses its primary branch entirely — acceptance then depends only on the covariance branch with coeff_cov's default 10, silently changing which residuals update the filter. With ds_scan_voxel=0, setLeafSize(0,0,0) makes pcl::VoxelGrid degenerate (PCL emits 'Leaf size is too small' and passes the cloud through unfiltered), silently changing point density and runtime. Both cases emit only a soft 'outside recommended range' WARN and continue. The listed defaults column for these params in PARAMETERS.md is '—', contradicting the header claim.

**Evidence.**

```
RESPLE.cpp:1725 'param.nn_thresh = CommonUtils::readParam<double>(..., "nn_thresh", 0.0);' with only a WARN at 1726-1729; RESPLE.cpp:1717-1722 same for ds_scan_voxel (default 0.0 -> setLeafSize(0,0,0)); Estimator.h:760 'if (abs(pt_data.zp) < pt_thresh || lid_cov < pt_data.var_pt*cov_thresh)'. PARAMETERS.md lines 4-5 'Every parameter is optional — omitting it keeps the listed default...'.
```

**Reference.** Upstream common_utils.h readParam(nh, name) (no-alternative overload) exited fatally on a missing required param — upstream_RESPLE.cpp used required reads for these; the local defaults changed missing-param behavior from fail-fast to silently degenerate.

**Suggested fix.** Give these parameters sane non-zero defaults (e.g. nn_thresh 0.5, ds_scan_voxel 0.2 to match every shipped config) or fail configuration when they are absent; fix the PARAMETERS.md 'every parameter is optional' claim for the '—' rows.

### 29. [LOW] Loc-gate covariance-inflation accumulator survives filter re-initialization — a fresh run publishes the previous run's tunnel inflation

- **Location:** `resple/include/Estimator.h:574`
- **Found by:** concurrency-fresh  ·  **Finder confidence:** 0.55
- **Verification:** _pending_

**Description.** loc_gate_cov_infl_ (and loc_gate_persist_ctr_, loc_gate_update_count_, loc_gate_VVt_infl_) are only ever decayed/grown by accumGateInflation() and are not touched by setState() (the comment at line 542-543 documents that deliberately only for cov_reset_, but the accumulators fall under the same omission). On a lifecycle re-cycle (deactivate->cleanup->configure->activate) initFilter() -> setState() rebuilds the spline and covariance from scratch, yet publishPoseAndTf immediately adds the stale loc_gate_cov_infl_ from the previous run to the new run's outgoing /odom translation covariance (RESPLE.cpp:2902-2904). With loc_gate_cov_rate enabled and the previous run ending in a degenerate stretch (steady state rate/(1-decay), e.g. the documented tunnel scenario), the new, healthy run reports metres-scale translation variance to the downstream EKF for the first ~100+ frames until the 0.99/frame leak drains it — misweighting fusion exactly when the operator restarted to recover. Only reachable when loc_gate_cov_rate > 0 (default off), hence low severity.

**Evidence.**

```
Estimator.h:574: Eigen::Matrix3d loc_gate_cov_infl_ = Eigen::Matrix3d::Zero();  // member-init only
Estimator.h:590-599 (accumGateInflation) is the sole writer besides the initializer — it only decays (*= loc_gate_cov_decay) or grows, never zeroes; setState() (Estimator.h:132-...) resets cov_sys/cov_rcp/a_mat but none of the loc_gate_* accumulators (verified in the current file; the setState diff removed cov_prior_ and added nothing gate-related).
Consumer RESPLE.cpp:2902-2904:
  P_pose.topLeftCorner<3, 3>() += if_lidar_only ? estimator_lo.locGateCovInfl() : estimator_lio.locGateCovInfl();
No reset on the re-activate path: on_cleanup (RESPLE.cpp:378-445) touches no estimator gate state, and initialization()/initFilter() (RESPLE.cpp:1994-2016, 3163) call only setState().
```

**Reference.** Commits 87f281f (publish-side inflation) and dc6afae (X-ICP gate); CLAUDE.md lifecycle re-entry discipline (hazards 17/18 pattern: run-scoped state must be re-initialized across configure/activate cycles).

**Suggested fix.** Add a resetLocGateState() (zero loc_gate_cov_infl_, loc_gate_VVt_infl_, loc_gate_VVt_, loc_gate_persist_ctr_, loc_gate_axes_/infl_axes_, loc_gate_armed_) and call it from setState(), or explicitly from initFilter().

### 30. [LOW] LO mode initialization now hard-blocks forever without an IMU topic (regression vs upstream LiDAR-only init)

- **Location:** `resple/src/RESPLE.cpp:3070`
- **Found by:** imu-path  ·  **Finder confidence:** 0.5
- **Verification:** _pending_

**Description.** The workspace-local change 'Always use IMU for gravity alignment, even in LO mode' (line 3058) removed upstream's 'if (!if_lidar_only)' guard around the IMU-based init. initialization() now unconditionally returns false until imu_init_num_samples_ (>=10, default 50) IMU samples arrive AND a window passes the variance check. A LiDAR-only bag or sensor rig with no IMU topic -- the primary use case LO mode exists for -- never initializes: the node spins forever emitting the throttled 'Waiting for 50 IMU samples' warning, publishing nothing. Upstream initialized LO with identity orientation immediately (upstream_RESPLE.cpp:626-647 runs the gravity block only when !if_lidar_only).

**Evidence.**

```
RESPLE.cpp:3056-3076: 'Eigen::Quaterniond q_WI = ...Identity(); // Always use IMU for gravity alignment, even in LO mode. ... { ... if (buff_size < imu_init_num_samples_) { imu_lock.unlock(); RCLCPP_WARN_THROTTLE(..., "Waiting for %d IMU samples for gravity alignment..."); return false; }' -- the block is unconditional; there is no timeout or no-IMU fallback to identity. All shipped LO configs happen to declare a Livox IMU topic, masking the regression.
```

**Reference.** upstream_RESPLE.cpp:626 'if (!if_lidar_only) {' gates the entire gravity/IMU block; upstream LO initializes with q_WI = Identity and zero IMU samples.

**Suggested fix.** In LO mode, fall back to identity q_WI (with a one-shot WARN that the map will not be gravity-aligned) after a bounded wait (e.g. 5 s without any IMU sample), or gate the IMU requirement on a new param (require_imu_init, default true).

## Coverage notes (what each finder checked and considered clean)

### spline-math

Audited /home/user/RESPLE/resple/include/Estimator.h (982 lines, full read) against refs/upstream_Estimator.h, the RESPLE paper (arXiv 2504.11580 §III-C1/IV-B), and refs/local_vs_upstream.diff; cross-read filter_health.h (full), the worker-loop IEKF/NIS/recovery/map-release/publish blocks of RESPLE.cpp (lines 580-1060, 1836-2030, 2871-2985), common_utils.h PointData/LidarConfig, Association.h findCorresp, SplineState.h getRCPs/updateRCPs, math_tools.h (diff vs upstream: only a benign Taylor-guard change), and doc/REVIEW_2026-06-27 A1-A4. Verified clean: propRCP prediction (a_mat recurrence matches upstream and the paper's random-walk-vs-extension transition; new pos = 2*p[n-1]-p[n-3], new ort-delta = delta[n-2], consistent between a_mat and addOneStateKnot; Q added as A P A^T + Q); IEKF relinearization (deltax = KH*delta_cur + K*innov - delta_cur is the standard iterated-EKF update about the prior, H re-prepped each iteration, cov_prop held at the prior across iterations — correct); Joseph form P+ = (I-KH)P(I-KH)^T + KRK^T with per-column K scaling by 1/R_inv (correct diagonal-R algebra), symmetrized at copy-out, and remains valid for the gate-projected suboptimal gain since KH is rebuilt from the projected K; Woodbury NIS identity nu^T R^-1 nu - b^T S^-1 b with b = H^T R^-1 nu against S = P^-1 + H^T R^-1 H (algebra checked, P = prior as required); small-measurement branch NIS from the directly-factored S; robust M-estimator (d2e4166) implemented as IRLS scaling of R^-1 (equivalent to sqrt(w) scaling of both H and r in the normal equations — consistent; weight w in [0,1], Huber/Cauchy formulas correct); range-dependent noise inflation for <3m points confirmed intentional (commit 3bffada message matches code); X-ICP-style gate (dc6afae): E_tt = sum n n^T of world-frame normals is normalized so eigenvalues sum to 1, VVt projector removes the LiDAR-column gain of the four world-frame RCP position row-blocks only (state layout [pos3, ort3] per knot verified via getRCPs/updateRCPs and a_mat), mask correctly distinguishes interleaved LiDAR/IMU columns in LIO, Joseph form stays consistent; covariance-gated map release (0e3192c) reads orientation trace under spline_mutex_ on the worker thread — frame and lock usage fine; publish-side inflation (87f281f) adds a world-frame 3x3 to the world-frame translation block of the published pose covariance with leaky-integrator persist/decay logic matching its comments; reinflateCovariance/A2 reinflates to nis_reset_cov*I set at configure for both estimators (correct direction, survives setState); getLastPoseCovariance G-matrix (2*rows-1..3 of Qleft(q^-1)) verified against quaternion left-multiplication algebra; getLastTwistCovariance rotates the world linear-velocity covariance R^T P R correctly; cov_rcp block indexing (24 RCP + ba@24 + bg@27) consistent across prepLiDAR/prepIMU/H assembly/a_mat with j<4 guards; zp sign convention (innov = -zp, zp = n.p_w + d) matches upstream; per-sample IMU R (cov_acc/cov_gyro variances, no discretization) is upstream-inherited design, not a drift; the 81*pd2^2 outlier gate matches upstream/FAST-LIO semantics with range_sensor correctly measured from the TF-derived sensor origin. Not re-reported: all 38 CLAUDE.md hazards and A2-A4 (spot-checked A2 as correct). Judgment calls not reported as bugs: zero-correspondence break after a successful iteration skips the cov_post_ copy-out (upstream-inherited, rare); detector's per-sample NaN breach vs per-window ratio asymmetry (documented design); uniform nis_reset_cov across pos/rot/bias units (tunable, documented); dual extrinsic paths (TF pre-transform + YAML q_lb) could double-apply if both configured, but the deployed path uses identity YAML extrinsics and upstream configs are reference-only.

### iekf

Audited resple/include/SplineState.h line-by-line against upstream (refs/upstream_SplineState.h), Sommer 2020, and the RESPLE paper. Verified clean: (1) blending + cumulative blending matrices — recomputed computeBlendingMatrix()/computeBaseCoefficients() independently in Python; both match the standard uniform cubic B-spline matrices exactly (blending = 1/6[[1,-3,3,-1],[4,0,-6,3],[1,3,3,-3],[0,0,0,1]] knot-row × u-power, cumulative = 1/6[[6,0,0,0],[5,3,-3,1],[1,3,3,-2],[0,0,0,1]]); derivative scaling via pow_inv_dt/inv_dt correct; the cumulative row-accumulation loop order is safe (adds not-yet-modified rows). (2) The A3 boundary fix in itpQuaternion/itpPose J_q and J_w: for size_J<4, idx_window = 2-idx_l = 4-size_J, so slot (4-size_J)+i is exactly the slot holding deque knot idx0+i; the per-slot chain-rule terms coeff[s]·Qright(suffix)·Qleft(prefix)·dexp[s] and the dw_dslot formulas are correct and reduce to upstream at size_J==4; test_spline_state.cpp pins them to central-difference numerical Jacobians at idx_l=0/1 (independent ground truth for the fix, with the position Jacobian as harness control). Note the tests would NOT catch a wrong blending matrix (all checks are self-consistent or differentiate the same matrix), which is why I verified it externally — it is correct, so no finding. (3) Angular velocity recursion w_j = A_j⁻¹w_{j-1} + 2·dλ_j·d_j matches the Sommer-style body-frame omega under the half-angle Quater::exp convention (rotation angle = 2|v|); dcoeff[0]=0 makes the omitted slot-0 term exactly zero. (4) Time normalization u=(t−start−idx_l·dt)/dt, idx_l/idx0/idx_r computation, maxTimeNs−1ns endpoint, and the defensive clamps (itpPose, prepareInterpolation) — consistent, no OOB in reachable states; cp0=q_knots[idx0−1] index provably ≥0 when idx_r>3. (5) pruneFrontKnots bookkeeping: start_t_ns advance in lock-step with deque erase keeps interpolation bit-identical (idle slide src=prune−3+j maps exactly to the old window inputs incl. cp0), totalKnots()/num_knots_pruned_ absolute-index protocol in getSplineMsg/updateKnots is coherent, ≥8 retention floor covers getRCPs (4) and the est-window (5); all confirmed by exact-equality tests including small/repeated prunes and both-sides-pruned protocol round-trips. (6) propRCP/a_mat extrapolation: new knot pos = 2c₂−c₀ (equals constant-velocity c_{n+1} for linear knots), delta = δ₁ — exactly the paper's A_s/A_r (Sec. IV-B) and identical to upstream; addOneStateKnot/updateRCPs quaternion re-chaining roots (q_idle[2] for local knot 0) survive pruning correctly (RcpRoundTripAfterPrune test). (7) Quater::exp/dexp: no double-cover hazard (half-angle map, small per-knot deltas; no log map exists in this codebase), dexp near-zero Taylor branch is the correct limit (J.row0=0, bottom=I); Qleft/Qright and the G matrix in getLastPoseCovariance (2·Im(q⁻¹⊗δq) rows of Qleft(q⁻¹)) verified by hand. (8) getSplineMsg start_idx/start_t contiguity logic incl. the pruned clamp and per-instance last_start_idx_ (fixes upstream's shared function-static). Minor non-findings noted but not reported: uninitialized cps[3] read is reachable only for num_knot==1 (clamped to u=0 where its coefficient is exactly 0; unreachable in practice — Estimator seeds 4 knots); getLastPoseCovariance comments claiming u≈1 weights [0,0,0,1]/[1,1,1,1] are inaccurate but code uses actual Jacobians; itpQuaternion's unconditional *w_out under if(J_w) is a null-deref trap with no current caller (inherited from upstream).

### association

Audited Association.h against vendor-point upstream and FAST-LIO, plus its call sites in Estimator.h (prepLiDAR/updateLiDAR*/update), SplineState.h (itpPose/itpQuaternion/prepareInterpolation), geometry_core.h (fitPlane), common_utils.h (esti_plane, PointData, LidarConfig), math_tools.h (drot/drotInv — only diff vs upstream is a benign Taylor-guard in exp), ikd_Tree Nearest_Search/Search, and RESPLE.cpp (worker deskew loop, collectMeasurements, all 7 sensor callbacks, mapIncremental, lasermapFovSegment, corresp_cfg wiring). Verified clean: (1) esti_plane threshold semantics — solve for n with b=-1, normalize, d=+1/|n|, threshold applied POST-normalization on true point-plane distance for all k neighbors, matching upstream/FAST-LIO; degeneracy guard is a relative colPiv pivot ratio, off by default; (2) residual zp = n.p_w + d with innovation -zp, H chain n^T*J_pos + n^T*drotInv(R_IL*p_b+t_bl, q)*J_ortdel where drotInv = d(R(q)v)/dq [w,x,y,z] — verified algebraically; correction is additive on raw knot params via updateRCPs, consistent with Jacobians taken w.r.t. knot position/ort_delta directly (no perturbation-side mismatch); bug-A3 slot mapping (4-size_J+i) present and correct in both itpPose and itpQuaternion; (3) deskew: pointBodyToWorld clamp to [minTimeNs,maxTimeNs] is correct (propRCP extends the spline past the newest point before deskew, so clamping only fires on genuinely stale/out-of-order stamps and increments the diagnostics counter — hazards 9/11 fixes are sound); per-point time offsets ns->float ms->int64 ns round-trip loses <10 ns; all 7 callbacks follow the intensity=ms convention; (4) nn gate: Nearest_Search max_dist is meters (squared internally, matching upstream's 2.236 call), local passes sqrt(nn_max_sq_dist), and the k-th-neighbor squared-distance re-check uses the squared param — consistent; (5) no hidden exactly-5-neighbor assumption: Nearest_Search resizes both output vectors to k_found, the size>=k check short-circuits the [k-1] index, fitPlane generalizes to point.size() with a <3 guard, mapIncremental bounds all neighbor loops; the reused thread-local pointSearchSqDis scratch is safe because Nearest_Search resizes it; (6) OpenMP findCorresp: thread-local scratch, disjoint pt_meas[i]/pt_neighbors[i] writes, per-thread stage counters merged with omp atomic, effect_num_k counted serially — no races; prepLiDAR parallel-for writes disjoint elements. collectMeasurements is line-identical to upstream. The range gate switch from pt_b.norm() to true sensor-frame range_sensor and the near-range noise inflation (commit 3bffada) are deliberate, self-consistent local features. Known hazards 1-38 and fixes A1-A4 were checked and not re-reported; the A1/A3 fixes themselves verified correct.

### imu-path

Audited the full IMU path in resple/src/RESPLE.cpp (getImuCallback, transformImu, updateImuTransform, worker drain, initialization/gravity alignment, collectMeasurements IMU windowing, initFilter) plus Estimator.h (prepIMU, updateLiDARInertial, update), math_tools.h, geometry_core.h, common_utils.h (g2R/R2ypr/ypr2R/ImuData), against upstream vendor-point sources and hand-derived rigid-body kinematics. Verified CLEAN: (1) the A4 fix (commit 0b9e5f2) is CORRECT for the transform direction actually used -- lookupTransform(base_link, imu_frame) yields T_base_imu, so r = translation = base->IMU vector in base frame, and the specific-force transfer f_base = R*f_imu - alpha x r - omega x (omega x r) requires the centripetal term to be SUBTRACTED, exactly as now coded; the alpha x r omission is acknowledged and acceptable (not measured); angular velocity R*omega and orientation composition q_world_imu * R_base_imu^-1 are also correct (orientation is unused downstream anyway). (2) Quaternion conventions: Hamilton throughout; Qleft/Qright standard; Quater::exp uses half-angle rotation-vector convention consistently with the spline's ort_delta; drotInv verified analytically as d(R(q)v)/dq and drot as d(R(q)^T v)/dq, used consistently in prepLiDAR (forward) and prepIMU (inverse); the G matrix in getLastPoseCovariance matches 2*imag(q^-1 x dq) rows of Qleft(q^-1); positify sign handling fine; no (w,x,y,z)/(x,y,z,w) constructor/coeff mixups found. (3) Gravity model: gravity = q_WI * (normalized mean accel * 9.81) = (0,0,+9.81) 'up', and prepIMU predicts RT*(a_spline + g) + ba, consistent with accelerometer-measures-minus-g at rest; g2R/yaw-zeroing matches upstream/VINS (the duplicated yaw removal in initialization() is redundant but harmless, same as upstream); best-quiet-window variance selection is a sound local improvement; hardcoded 9.81 magnitude matches the worker's acc_ratio scaling factor. (4) Bias handling: ba/bg are estimated (XSIZE=30) with measurement model z = h + bias, H bias blocks = I, sign correct and identical to upstream; the 10 m/s^2 / 5 rad/s residual gates match upstream. (5) Units/timestamps: acc_ratio scaling is applied consistently at the drain and in the health monitor (except the transformImu ordering bug reported); gyro assumed rad/s (standard ROS, no upstream ratio param either); IMU time from header.stamp via rclcpp::Time(...).nanoseconds() with no lossy conversions; collectMeasurements imu_meas sliding-window trim (maxTimeNs - knot interval) is byte-identical to upstream. A1/A2 fixes were re-verified as correctly implemented (per-frame last_nis_ NaN reset in all updateIEKF* overloads; cov_reset_ defaults to 1.0*I with setRecoveryCovariance plumbing). Did not re-report any of the 38 known hazards or A1-A4; the Q bottomRightCorner finding, the acc_ratio/transformImu units interaction, the pre-init frame gap, and the LO-init IMU dependency appear in none of them.

### node-logic

Audited /home/user/RESPLE/resple/src/RESPLE.cpp (all 3634 lines read) against refs/upstream_RESPLE.cpp, plus the SplineState.h/Estimator.h/Association.h/common_utils.h surfaces it calls, per the assigned dimension. Verified clean: (1) processData drain ordering and the Phase-2.6 under-lock emptiness check; collectMeasurements is line-for-line faithful to upstream (same window logic, same <=max_time_ns / numKnots<10 / maxTimeNs-dt_ns boundaries; the local pt_meas-empty guard at line 688 is an improvement over upstream's unguarded pt_meas.back()). (2) est_window publish gate: totalKnots()-space max_spl_knots, getSplineMsg absolute start_idx hint with num_knots_pruned_ clamp and last_start_idx_ contiguity all match the upstream contract; knot_rot_checked_ is kept in absolute space and correctly converted with -pruned_k (clamped); Estimator's getLastPoseCovariance/getLastTwistCovariance use consistently LOCAL indices (J.start_idx vs numKnots()-4); found no local index stored across cycles or unconverted absolute deque access. (3) NIS recovery: HOLD gates all four pose outlets (pose, pose_cov, odom, TF) through the single publishPoseAndTf choke point; est_window/current_scan continue by documented design; RESET reinflates via cov_reset_ (A2 fix verified present) and resets the detector. (4) Drop-oldest caps: pushScanBounded pops pc_buff+t_buff in lock-step from the front with counters; IMU staging cap erases begin(); monotonicity gates (last_t_ns) unaffected; downstream propRCP handles the gaps. (5) Publish paths: twist correctly in child (body) frame per REP-105 (v_body = q^-1 v_world; omega from itpQuaternion is body-frame), twist covariance rotated into body for the linear block and body-native for angular; 6x6 covariance row-major mapping in [pos, rot] order is correct; loc-gate inflation (87f281f) is built from world-frame plane-normal eigenvectors and added to the world-frame translation block of the pose covariance - frames consistent. (6) map_insert_lag staged-release logic (0376db2/0e3192c): stage/release/pop-front bookkeeping consistent, released deskew under spline_mutex_, cov-gate early release sound; only weakness is the time-sortedness assumption breaking mildly under multi-lidar (delayed release only, off-by-default feature - not reported). (7) Timestamp arithmetic: all spline/scan bookkeeping is int64 ns; the float ms round-trip via .intensity loses <10 ns; Hesai modf double-seconds loses ~0.5 us (upstream-identical); dt_ns=1e9/knot_hz exact for canonical rates - no 2^53 hazards found. Cross-checked all 38 CLAUDE.md hazards and A1-A4 (verified A1 last_nis_ reset, A2 cov_reset_, A4 centripetal sign are actually fixed in the code) and did not re-report them; thread-safety races (heartbeat pc_buff read without mtx_pc, unlocked spline reads on the worker) were left to the concurrency agent.

### mapping-node

Audited the Mapping-node + map-management dimension against upstream vendor-point sources and FAST-LIO2 heritage. Verified clean: (1) lasermapFovSegment (RESPLE.cpp:3323-3397) is line-faithful to upstream/FAST-LIO2 — MOV_THRESHOLD*det_range edge test, mov_dist formula, per-axis slab removal all match; hazard-25 ::lowest() fix is correct and I found no min()/lowest() siblings; the removed slab is always on the trailing side (points ahead of a fast-moving robot are not deleted; the inherited fabs() outside-cube quirk matches upstream). (2) pruneMapRadius (§3.4): 2×det_range floor, 10% movement hysteresis, and geometry_core subtractBox (disjoint slab decomposition of outer\keep) are correct; lock discipline (mtx_map_ unique in the async task, spline_mutex_ taken inside lasermapFovSegment in mtx_map_→spline_mutex_ order) matches the documented ordering. (3) mapIncremental matches upstream RESPLE bit-for-bit (0.866 first-neighbor gate, 0.5-voxel need-add test, floor-based voxel center); hazard-32 NaN skip keeps pc/nearest_pts index pairing intact. (4) The buffer-swap protocol, 5s chunked future wait, and the async lambda try/catch+pending-clear are consistent; the §6.3 map_insert_lag staging path (RESPLE.cpp:836-893, 966-979) keeps released_w_[i] ↔ map_insert_staging_[i] correspondence and pops in order. (5) Replica ingest: Option B single-writer discipline on spline_active_ holds (init/drain/prune/publish/processScan all on the worker); lock order m_spline→maps consistent in both branches; getSplineMsg contiguity (min with last_start_idx_+1, publish-before-prune) guarantees window overlap absent drops, and the a84b7f4 lossless queue removes the overwrite drift — the residual defect is the gap time-base (finding 2). Replica pruning needs no mirroring thanks to absolute indexing; idle-copy gating and setIdles/addOneStateKnot cumulative-q reconstruction converge to the sender's knots. (6) fb069df time-paired TF: pubOdom composition T_map_odom = T_map_base(tip) * T_base_odom(tip) is algebraically correct including invert_tf, the Time(0) fallback is warm-up-only, and future-dating via map/transform_tolerance is applied on the stamp only. (7) pubOdom twist (body-frame velocity, shortest-path quaternion delta) is correct. (8) The REVIEW doc's A1-A4 and all 38 CLAUDE.md hazards were checked against my findings to avoid re-reports; the doc's note on getSplineMsg start_t/start_idx mismatch (\"benign, index-addressed\") is a different issue from finding 2's gap time-base. Not verified by execution: no bags available in this session; all findings are from source analysis against the staged upstream references.

### ikdtree

Audited /home/user/RESPLE/resple/include/ikd-Tree/ikd_Tree.cpp (1738 lines) and ikd_Tree.h in full against both references. Vendor-point (refs/upstream_ikd_Tree.cpp/.h) vs current hku-mars main (refs/ikd_Tree_upstream.cpp/.h): a full diff shows only formatting changes plus RESPLE-side additions (radius_sq, Search_by_radius/Radius_Search, removed dead vars) — no post-vendor upstream bugfixes are missing. Checked clean: (1) atomic conversion — the only compound update on an atomic node field is '(*root)->invalid_point_num += 1' (Delete_by_point line 967), a genuine atomic fetch-add; Update()/Delete_by_range/Add_by_range/run_operation all compute from other fields and store, no load-op-store lost-update on the same atomic; (2) Push_Down ordering — child flags are written before the parent's need_push_down_to_* clear in all four branches (clears at 1378/1404/1432/1458), and the lock-free fast pre-check (1344) is safe under seq_cst atomics; the pre-lock capture of operation.tree_deleted (1348) matches upstream placement; (3) Criterion_Check thresholds/logic identical to upstream; torn multi-field TreeSize reads are confined to mutator callers, which are fully serialized by the Phase 2.5 whole-op recursive working_flag_mutex, so no divergent rebuild decisions; (4) Nearest_Search kNN max-heap (cap 2k, pop-when-full, tie-break on point.x) and Search() pruning bound identical to both references; result fill-backwards is equivalent to upstream insert-at-begin; (5) box searches keep upstream's half-open [min, max) boundary semantics and full-cover conditions; Search_by_radius sphere prune/include bounds are mathematically correct for the box half-diagonal radius_sq; (6) Delete_Point_Boxes/Add_Points interplay with the FOV caller is serialized at the RESPLE level by mtx_map_, and Add_Points' internal Search_by_range acquires working_flag then search_rw shared — the same order as the rebuild thread, so no inversion there; (7) memory — STATIC_ROOT_NODE leak fix in ~KD_TREE and Build() is correct (Root_Node freed first, then the static root), delete_tree_nodes destroys per-node mutexes, rebuild path frees the old subtree via delete_tree_nodes(&old_root_node); the Eigen alignment macros in ikd_Tree.h are made globally consistent by matching PUBLIC compile definitions in CMakeLists.txt (lines 174-175, 289-290), so no cross-TU allocator mismatch. Known hazards 1,22,33,34,35 were not re-reported; findings 1 and 2 are incompleteness in the hazard-34 and hazard-35 fixes respectively (reader-side lock inversion via Push_Down; empty-flatten swap bypassing the whole-op mutex), and finding 4 is a residual non-atomic Rebuild_Ptr race the Phase 2.4 atomic conversion missed. Upstream-quirks intentionally not reported (present identically at the vendor point, no local semantic claim violated): working_flag left true on box-miss early returns, rebuild_flag cleared outside working_flag (folded into finding 4's fix note), stale-ancestor Update-walk skip when new_root_node is null, uninitialized alpha_bal/alpha_del on non-root nodes (only read by uncalled root_alpha), and the uncalled racy accessors validnum/tree_range/root_alpha (noted inside finding 4).

### recent-commits

Reviewed every listed commit's full diff plus the surrounding current-tree code. Verified clean: (1) A3 Jacobian window-slot fix — derived the slot mapping (entry i → slot (4-size_J)+i; idx_window == 4-size_J since size_J = idx_l+2 for idx_l<2) for J_q in both itpQuaternion and itpPose and for the new dw_dslot[] J_w refactor (dw3/d(delta_s) chain terms match; interior size_J==4 path is bit-identical to the old code), and ran the repo's ROS-free suite — all 49 tests pass including SplineJacobianBoundary.RotationJacobiansMatchNumericalNearStart which numerically checks J_q and J_w at idx_l=0 and idx_l=1. (2) A4 centripetal sign — transform is lookupTransform(base_link ← imu), so translation r = base→IMU in base coords, ω rotated into base; a_origin = R·a_imu − ω×(ω×r) is the correct rigid-body removal; subtraction is right. (3) A2 — reinflateCovariance targets the new cov_reset_ (not touched by setState), nis_reset_cov read via guarded readParam at configure and set on both estimators; doc/PARAMETERS.md consistent. (4) dc6afae gate — E_tt built from unit normals (fitPlane normalizes, so eigenvalues sum to 1 as documented), per-point normalization by accepted-row count, mask column semantics correct for the LIO timestamp-interleaved rows (rejected rows have zero K columns so their mask value is inert), projector applied to the 4 RCP position row-blocks only, KH rebuilt from the projected gain in both update() branches, Joseph form valid for arbitrary gain, IEKF deltax formula intact, armed flag reset lifecycle safe across iterations/frames, NIS computed pre-projection (correct). (5) 87f281f — leaky-integrator math matches the documented rate/(1−decay) steady state, deficit weighting, stricter threshold derivation (gate/10), publish-side-only addition in publishPoseAndTf (worker thread, internal cov untouched), diag export threading OK; noted but did not report (too minor): stale loc_gate_infl state persists across a zero-correspondence frame, and loc_gate_cov_infl_ is not reset on a lifecycle re-configure (decays in seconds, feature default-off). (6) d2e4166 robust kernel — IRLS weight applied to R_inv consistently in both assemblers; weights never zero (no inf in the Joseph K·R·Kᵀ term); 'none' is bit-exact legacy. (7) ded552b localizability diagnostic — report-only, q_edge under spline_mutex_, E_rr lever arms in body frame. (8) 0e3192c — cov-gated early release and knot-rotation check verified for prune-offset indexing (knot_rot_checked_ clamped to pruned count, deque index = absolute − pruned) and negative loop bounds at startup. (9) 0376db2 insertion lag — staging/release pairing (released_w_[i] ↔ staging front pops), std::move double-use safe, cleared in on_cleanup, release horizon ≤ maxTimeNs so the deskew clamp never fires. (10) a84b7f4/ffac28c — queue drain fully under m_spline with no lost-wakeup on spline_pending_ready_, gap counters. (11) fb069df/be99a7e — pubOdom TF composition frames verified (lookupTransform(base←odom) at tip time composed as T_map←base·T_base←odom), path-tip lag, ns stamp units in pcl headers confirmed. (12) Lifecycle: new state (map_insert_staging_, released_w_, knot_rot_checked_, nis_hold_active_) reset in on_cleanup. Cross-feature interactions checked: gate+robust kernel (independent, weights don't feed E_tt — a design choice, not a bug), gate+NIS hold/reset (no double-count; hold suppresses the publish that carries the inflation), inflation+hold. Known-pre-existing, not reported per instructions: NIS dof counts zero-innovation rejected rows and IMU rows (mild low-bias of the windowed mean, Phase 3.3 era, not in the audited commits); upstream-inherited IEKF quirk where a mid-loop num_tot_eff==0 break skips the cov_post_ copy-out (identical in upstream); getImuCallback holds m_buff across a 0.1 s canTransform wait during warm-up (pre-dates the audited range).

### config-units

Audited the configuration/parameters/units dimension. Clean areas verified: (1) CMakeLists.txt — ENABLE_TSAN/ENABLE_ASAN mutual exclusivity enforced via FATAL_ERROR (line 34); EIGEN_INITIALIZE_MATRICES_BY_NAN correctly Debug-only via $<CONFIG:Debug> genexpr (line 188); sanitizer -O1 flags win over Release -O3 (target options come after CMAKE_CXX_FLAGS_RELEASE); ENABLE_EIGEN_BLAS toggle matches PARAMETERS.md's A/B table; BLAS linked into all Eigen-using test targets as documented. (2) readParameters()/on_configure/on_activate vs PARAMETERS.md: num_threads=5, num_match_points=5, num_points_upd=100, n_iter=1, cube_len=1000, spline_prune_keep_knots=600 (clamp <100 -> 100), max_scan_buffer=0, max_imu_staging=2000, nis_window/warn/diverged/breach=32/2/4/3, nis_recovery_mode off, nis_reset_cov=1.0, map_prune_radius=0 (warn vs 2x det_range=100 correct with ::lowest already fixed), robust_kernel none/delta 0.1, loc_gate_* defaults, map_insert_lag_knots=0, map_deskew_lag_knots=8 (both Mapping read sites), map/transform_tolerance=0 — all match the doc. cov_P0/std_sys scaling by dt_s^2 and Q assembly match upstream_RESPLE.cpp exactly. (3) q_lb array parsing order (w,x,y,z via .at(0..3)) matches upstream and the doc; t_bl = q_lb^-1(-t_lb) matches upstream. (4) lidars map keyed by lidar.type matches upstream (duplicate-type limitation is inherited, not a local regression). (5) per-sensor callbacks: Ouster t/1e6 ms, Livox offset_time/1e6 ms, Hesai modf-based double-seconds->ms, Mid360Boxi double-ns->ms conversions all internally consistent; monotonic last_t_ns gates consistent; lidar_time_offset applied (subtracted, ns) only in Ouster+generic paths, same scope as upstream. (6) point_cloud_adapter byte-offset/endianness math (memcpy + reverse) correct; float32-epoch precision loss correctly avoided by double-ms normalization before cast. (7) launch files: use_mapping default false wiring + LogInfo consistent across live-sensor launches; config_file arg only on the three live launches as documented. (8) has_parameter guards on num_threads/num_match_points/lidars re-declare paths present. Findings not re-reported: all 38 CLAUDE.md hazards and A1-A4 (checked doc/REVIEW_2026-06-27, including its lower-severity notes). Main uncertainty flagged in confidences: whether NTU/KTH bags carry a base_link TF (finding 2) and the exact livox_ros_driver2 PointCloud2 field layout (finding 3).

### numerics

Swept for the requested numerical-robustness dimension with upstream (vendor-point 3d0e78c) cross-checks: Estimator.h in full (propRCP a_mat structure matches upstream 2*c2-c0 / knot-1 ort-delta propagation; information-form update with LLT info() checks and FullPivLU isInvertible in the small branch; Woodbury NIS identity verified algebraically; Joseph-form KR column scaling and HT_R_inv rowwise broadcast verified; loc-gate projector math incl. Ett normalization, deficit weighting, applyLocGate masking — lidar_part is eagerly evaluated so no aliasing; getLastPoseCovariance's G = 2*imag(q^-1 (x) dq) rows verified component-by-component against quaternion algebra and the [w,x,y,z] Jacobian43 ordering). SplineState.h in full (A3 slot remapping verified correct for size_J<4 and identical to upstream for size_J==4; prepareInterpolation/itpPose clamps, u computed from relative int64 ns so no 2^53 epoch precision issue; pruneFrontKnots idle-slide index math; blending/base-coefficient generators incl. pow(0,0)=1 case; found the itpQuaternion q_out contract hole reported above and empirically confirmed it with a compiled repro). Association.h (pointBodyToWorld clamp, findCorresp k-NN gates, FAST-LIO-style 81*pd2^2 range gate uses sensor-frame range correctly). geometry_core.h (fitPlane normal-equation solve: d=1/|n| Hessian normalization correct vs FAST-LIO esti_plane; zero-norm and non-finite guards present; subtractBox tiling logic checked). filter_health.h (Welford stats, gyro RMS formula E[g^2]=mean^2+var, window replace-vs-latch semantics, NIS detector windowing — thresholds are heuristic ratios of dof, no chi-square constant misuse; found the dof dilution reported above at the producer side). math_tools.h (Quater::exp/dexp small-angle branch at 1e-10 with correct Taylor fallback via boost sinc_pi; Qleft/Qright/drot/drotInv match upstream bit-for-bit). point_cloud_adapter/ingest (absolute-epoch time kept in double ms until min-subtraction before float cast — explicitly handles the float-truncation trap; int-ns/float-s auto units sane). RESPLE.cpp math paths: gravity init (windowed min-variance scan, g2R yaw removal — VINS-standard; zero-accel normalized() edge is Eigen-safe returning zero, degrades but no NaN), time conversions (all per-point offsets relative before float; time_offset is int64; dt_ns=1e9/knot_hz exact for the shipped 100 Hz), localizability E_tt/E_rr diagnostics, lasermapFovSegment (lowest() fix in place, hazard 25), pruneMapRadius box math, mapIncremental NaN skip (hazard 32), publishPoseAndTf covariance plumbing (led to finding 1). Mapping.cpp: deskew ns conventions (pcl header.stamp consistently ns by local convention), finite-difference twist with dt guard and shortest-path AngleAxis. Checked and did NOT re-report: all 38 CLAUDE.md hazards and REVIEW A1-A4 (verified the A3 fix is correct; A1's fix is incomplete only in the dof-dilution sense reported as finding 2, which is a distinct producer-side defect, not a regression of A1 itself). Deliberately not reported: near-range noise_scale downweighting direction (deliberate, commented, defensible), prepareInterpolation uninitialized cps[3] for num_knot in {0,1} (upstream-identical, unreachable through guarded callers), IMU-frame vs base_link gravity-alignment extrinsic question (frames/TF semantics, outside this dimension and config-dependent), 1/cov_acc division (config-validated nonzero in all shipped YAMLs).

### concurrency-fresh

Scope: concurrency of code added after the hardening passes (fd77ab1..HEAD: A1-A4 fixes, MapSaving, X-ICP gate dc6afae, covariance inflation 87f281f) plus the older local features (robust kernel d2e4166, localizability diag ded552b, cov-gated release 0e3192c, insertion lag 0376db2, est_window ingest a84b7f4, deskew lag/dedup be99a7e, TF pairing fb069df). Checked clean: (1) X-ICP gate + inflation state (loc_gate_mask_/VVt_/armed_/update_count_/cov_infl_) — all written inside updateIEKF* under the worker's mtx_map_(shared)+spline_mutex_ block and read only on the worker (publishPoseAndTf at RESPLE.cpp:2892-2904, diag snapshot at 955-964) or via the three new relaxed atomics consumed by the executor-thread updateDiagnostics — no cross-thread unlocked access; applyLocGate mask sizing/guard (mask.size()==K.cols()) verified for both LO (all-ones over num_valid) and LIO (interleaved-row marking) assemblers. (2) A1 NaN-reset and A2 reinflateCovariance both execute on the worker only (cov_rcp is worker-thread-only by contract; recovery switch at RESPLE.cpp:789-834). (3) publishPoseAndTf/publishCurrentScan/heartbeat spline reads are worker-thread reads against a worker-only writer (async lambda reads under spline_mutex_ per hazard 37 fix; lock order mtx_map_->spline_mutex_ preserved at 3334-3350); getLastPoseCovariance call sites at 874/1181 correctly under spline_mutex_. (4) Insertion-lag staging (map_insert_staging_/released_w_), cov-gated release, knot under-resolution check: all inside the worker's spline_mutex_ block (838-926) or worker-local; diagnostics exported via atomics (knot_rot_max_mrad_ CAS loop correct). (5) Mapping est_window ingest: est_window_queue_/est_windows_dropped_ under m_spline in both getEstCallback and the drain; spline_active_ mutation is worker-only (Option B), publish block holds m_spline->maps in the corrected order; processScan pulls clouds fully under the per-map mtx (front/pop UAF fix intact) and the OMP transformCloud reads baselink_to_imu_ after a write-once latch with a happens-before through the buffer mutex. (6) MapSaving locking itself is correct (accumulated_map under mtx_map, copy-then-write outside the lock; single-threaded spin) — only the exception escape was flagged. (7) getImuCallback/imu_health_/imu transform cache: all under m_buff; updateDiagnostics reads only atomics or takes m_buff. (8) pushScanBounded cap + drop counters under mtx_pc + atomics. (9) ikdtree.size() in the Phase-4 diag message is the hardened shared-lock version. (10) Lifecycle: on_cleanup resets checked member-by-member; besides the two reported gaps (last_pose_pub_time_ns_, loc-gate accumulators) the remaining unreset members are cumulative diagnostics counters (intentional) or rebuilt in on_configure (nis_detector_, imu_health_). Did not re-audit the 38 fixed hazards or upstream ikd-Tree internals beyond confirming the hazard-36 fix boundary; the A3 Jacobian slot math and A4 sign fix were sanity-checked but belong to the math dimension.

