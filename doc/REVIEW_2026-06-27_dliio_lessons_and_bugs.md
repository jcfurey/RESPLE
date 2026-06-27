# RESPLE review — dliio degeneracy lessons + bug hunt (2026-06-27)

Status: **bugs A1–A4 fix-in-progress**; optimizations B1–B7 are recommendations, not yet
implemented.

This document is the RESPLE-side write-up of a cross-analysis against the sibling
`dliio` estimator. dliio and RESPLE are both B-spline / recursive LiDAR-inertial
odometry estimators that have been benchmarked on the **same Ouster tunnel bag**
(`06042026_`, a chaotic "map-lock" basin). dliio's `doc/FINDINGS_2026-06-*.md`
series is, in effect, an n≥24-validated experimental program telling us which of
RESPLE's robustness mechanisms to enable and how. **Nothing in `src/dliio` was
modified** producing this report; the dliio docs are cited read-only.

References (read-only, in `src/dliio/`):
- `doc/FINDINGS_2026-06-22.md` — the "dilution wall"; the tunnel degeneracy is **rotational (yaw)**, not translation
- `doc/FINDINGS_2026-06-25.md` — X-ICP ternary gate **validated n=24, Fisher p=0.0055**
- `doc/FINDINGS_2026-06-26.md` — observer-gain freeze is catastrophic (5/5 DIV); X-ICP stack-fragility
- `results/SESSION_2026-06-24_governor_validated_n24.md` — governor pose-clamp **validated n=24, ~130× worst-case reduction**
- `include/dlio/degeneracy_governor.h`, `include/nano_gicp/xicp_localizability.h`, `include/dlio/degeneracy_observer.h`

---

## 0. Headline: RESPLE's degeneracy stack is built but disabled

RESPLE already contains a full degeneracy/robustness stack, all **off in every shipped
config** (`config/config_*.yaml`, including `config_06042026.yaml`, the config for the
exact tunnel bag dliio fights):

| Mechanism | Code | Param | Default |
| --- | --- | --- | --- |
| Translation localizability gate | `Estimator.h::armLocGate` / `applyLocGate` | `loc_gate_trans_min_eig` | `0.0` (off) |
| Publish-side covariance inflation | `Estimator.h::accumGateInflation` | `loc_gate_cov_rate` | `0.0` (off) |
| Robust M-estimator kernel | `Estimator.h::robustWeight` | `robust_kernel` | `none` |
| Plane-fit degeneracy guard | `geometry_core.h::fitPlane` | `plane_min_cond_ratio` | `0.0` (off) |
| NIS divergence detector + recovery | `filter_health.h`, `RESPLE.cpp` | `nis_recovery_mode` | `off` |

The dliio findings (next section) say which of these to turn on, and identify
gaps where RESPLE's version would not catch the failure mode dliio actually
characterised.

---

## Part A — Bugs found

All four were verified against the source; line numbers are against the `lyrical`
branch at the time of writing. Severity reflects the deployed LIO / CPU path.

### A1 — NIS detector fed a STALE NIS on zero-correspondence frames — **MED**

- **Where:** `RESPLE.cpp` worker loop (the `nis_detector_.feed(...)` block, ~L764–833);
  `Estimator.h` `updateIEKF*` (the `num_tot_eff>0 ? update() : break` structure) and
  `update()` (only writer of `last_nis_`).
- **Root cause:** `last_nis_`/`last_nis_dof_` are written *only inside* `update()` and are
  never reset per frame. When a frame yields **zero valid correspondences**, the IEKF
  breaks before calling `update()`, so `lastNis()` returns the *previous* frame's
  (healthy ≈1.0) value. The worker feeds that stale value to the detector every cycle
  (the block runs whenever `pt_meas` or `imu_meas` is non-empty).
- **Impact:** lost-correspondence (open space / featureless stretch / outrunning the
  map) is a primary LIO divergence trigger — and is exactly the case the detector goes
  blind on. `filter_health_state_` reads OK while the filter dead-reckons, and
  `nis_recovery_mode=hold/reset` never engages. The code comment ("NaN when the update
  was skipped — counted as a breach") is true only for the *numerical-failure* skip,
  not the zero-correspondence skip.
- **Fix:** reset `last_nis_ = NaN`, `last_nis_dof_ = 0` at the top of each `updateIEKF*`
  call (matches the existing "pessimistic default" pattern inside `update()`). A frame
  that never calls `update()` then feeds NaN → the detector's non-finite path counts a
  breach, as intended.

### A2 — `nis_recovery_mode=reset` DEFLATES the covariance instead of reinflating — **MED** (off by default)

- **Where:** `Estimator.h::resetCovarianceToPrior` (`cov_rcp = cov_prior_`); `cov_prior_`
  set in `setState` from the startup prior; `RESPLE.cpp` initFilter
  (`cov_P0 = 0.02 * dt_s²`, `cov_RCPs = cov_P0 * I`); the RESET branch in the worker.
- **Root cause:** `cov_prior_ = cov_P0·I = 0.02·(0.01)²·I = 2e-6·I` (translation σ≈1.4 mm) —
  the *tightest* covariance the filter ever holds (the startup anchor). NIS divergence
  signals an **over-confident** (too-small) covariance; resetting to 2e-6·I makes it
  *more* over-confident → Kalman gain → 0 → measurements ignored → the filter freezes on
  the diverged state, advertising mm-scale `/odom` covariance while the true error is
  metres. That is the exact outcome the comment says it prevents.
- **Fix:** reinflate to a genuine recovery covariance, not the startup anchor. Introduce
  `cov_reset_` (default `1.0·I` ≈ σ 1 m / 1 rad), a `setRecoveryCovariance(var)` setter,
  and a `nis_reset_cov` node param (default 1.0). Rename `resetCovarianceToPrior` →
  `reinflateCovariance` for honesty.

### A3 — Spline orientation + gyro Jacobians MISINDEXED at the spline-start boundary — **MED** (low reachability)

- **Where:** `SplineState.h` `itpQuaternion` (J_q loop + J_w unrolled branch) and
  `itpPose` (J_q loop).
- **Root cause:** the position Jacobian maps window slot → knot via
  `J_p->d_val_d_knot[i] = coeff_p[4 - size_J + i]` (slot `idx_window+i`). The rotational
  Jacobians instead index slot `i` directly (`coeff[i] * Qright(q_r_all[i]) *
  Q_l_all[i] * dexp_dt[i]`, and the gyro branch hard-codes slots 1/2/3). When
  `idx_window = max(0, 2 - idx_l) > 0` (query within the first 2 knot-intervals of the
  spline start), the real knots occupy slots `[idx_window, 4)`, so indexing by `i` reads
  *idle-knot* derivatives, shifts the attribution by `idx_window` knots, and drops the
  last real knot's contribution. The consumer (`prepLiDAR`/`prepIMU`) pairs position and
  orientation entries at the same index, so the orientation/gyro block of the
  measurement Jacobian is corrupted.
- **Reachability:** only `numKnots ≤ 5`, i.e. the **first ~2 IEKF updates after each
  `init()`** (once growth/pruning advances `start_t_ns`, in-window queries have
  `idx_l ≫ 2` → `idx_window = 0`, the size_J==4 path, which is correct). Self-heals;
  re-triggers on every respawn. No OOB (wrong values, not a crash). Covariance
  diagnostics (queried at `maxTimeNs()`, `idx_window=0`) are unaffected.
- **Fix:** index the rotational loops by `4 - size_J + i` (= `idx_window + i`), mirroring
  the position loop; rebuild the gyro branch as a slot-indexed `dw_dslot[4]` array then
  assign `J_w->d_val_d_knot[i] = dw_dslot[(4-size_J)+i]`. The `size_J==4` path is
  bit-identical to today. Add a numerical-Jacobian boundary unit test (none of the
  existing tests exercise `size_J<4` — they compare full-vs-pruned at the tail only).

### A4 — `transformImu` adds the centripetal lever-arm term with the WRONG SIGN — **LOW** (≈1 cm lever arm here)

- **Where:** `RESPLE.cpp::transformImu` (`R·a_imu + ω×(ω×r)`).
- **Root cause:** to refer the IMU accel to the base_link origin the rigid-body relation
  is `a_origin = a_imu − α×r − ω×(ω×r)` — the centripetal term must be **subtracted**.
  As written it doubles the term instead of removing it, injecting `+2·ω×(ω×r)` into
  every fused accel sample during turns.
- **Impact:** for the deployed Ouster the IMU offset is r ≈ [−0.0024,−0.0097,0.0075]
  (≈1.25 cm, identity rotation), so the magnitude is ~0.05–0.1 m/s² at a few rad/s —
  small but systematic, growing as ω². Matters more on platforms with a larger IMU
  offset. Live in the LIO IMU path.
- **Fix:** flip the sign (`−`) and correct the comment. The `α×r` omission is acknowledged
  and acceptable (angular acceleration is not measured).

### Lower-severity notes (verified, not scheduled)

- **CUDA path** (off by default, otherwise well-hardened — the 2.4 m search coverage vs
  the 2.236 m gate is provably correct): `cuda_knn.cu::insert_topk_pt` does not guard NaN
  *map* points the way it guards NaN queries (can only *lose* a correspondence near the
  origin, never create a wrong one); CUB device-call return codes are discarded
  (misattributes a CUB error to a later `cudaMemcpy`). **No unit test exists for the GPU
  path** — the highest-value gap if CUDA is ever deployed.
- **Mapping node** (non-production, `use_mapping:=false`): `getSplineMsg` ships
  `start_t` = time of knot `total-5` while `start_idx` can be smaller — latent/benign only
  because the replica is index-addressed, not time-interpolated.
- **LO mode** silently ignores the `nis_*` params (configured only under
  `if (!if_lidar_only)`); harmless since defaults match.

---

## Part B — Fix checklist

- [x] **A1** — `Estimator.h`: reset `last_nis_`/`last_nis_dof_` at the top of all four
      `updateIEKF*` overloads (LO, LO-CUDA, LIO, LIO-CUDA). *Done — applied after
      `corresp_stats.reset()` in each overload.*
- [x] **A2** — `Estimator.h`: add `cov_reset_` + `setRecoveryCovariance()`, rename
      `resetCovarianceToPrior` → `reinflateCovariance`; `RESPLE.cpp`: read `nis_reset_cov`
      (default 1.0), plumb to both estimators, update the two RESET call sites; add the
      param to `config_06042026.yaml` with a comment. *Done.*
- [x] **A3** — `SplineState.h`: fix the slot indexing in the three rotational Jacobian
      loops (`itpQuaternion` J_q + J_w, `itpPose` J_q); add a numerical-Jacobian boundary
      test in `test/test_spline_state.cpp`. *Done.*
- [x] **A4** — `RESPLE.cpp`: flip the centripetal sign in `transformImu`; fix the comment.
      *Done.*
- [~] **Validate** — see below. Unit suite green; `Estimator.h` syntax-clean against the
      real ROS+PCL toolchain. Full colcon build + tunnel-bag replay still pending the
      container.

### Validation done (2026-06-27, host `/opt/ros/lyrical` + g++/Eigen/GTest/boost/PCL)

- **A3 — definitive.** New test `SplineJacobianBoundary.RotationJacobiansMatchNumerical-
  NearStart` builds a 5-knot spline, queries at the start boundary (idx_l ∈ {0,1},
  size_J ∈ {2,3}), and compares analytic J_q/J_w against central-difference numerical
  Jacobians (position Jacobian as a harness control). It **passes with the fix** and
  **fails against the original `HEAD` header** (buggy gyro Jacobian off by ~6×: analytic
  150 vs numerical 25, J_w norm error 125). The fix's `size_J==4` path is unchanged, so
  the existing prune/protocol tests still pass.
- **Regression** — `test_spline_state` (11/11, incl. the new test), `test_geometry_core`
  (16/16), `test_filter_health` (12/12) all green.
- **A1 + A2** — `Estimator.h` compiles clean (`-fsyntax-only`) against the real ROS +
  PCL headers; no dangling `cov_prior_` / `resetCovarianceToPrior` references remain; the
  new symbols (`reinflateCovariance`, `setRecoveryCovariance`, `cov_reset_`,
  `nis_reset_cov`) resolve in both `Estimator.h` and `RESPLE.cpp`.
- **Pending** — full `colcon build --packages-up-to resple` (validates `RESPLE.cpp`'s
  A2/A4 in the link build; the canonical build is in Docker per `CLAUDE.md`) and a
  tunnel-bag replay smoke (`config_06042026.yaml`) to confirm no behavioural regression.

---

## Part C — Optimization roadmap from dliio (recommendations, ranked)

These reuse RESPLE's existing (dormant) machinery; each is backed by dliio's n≥24 work.

- **B1 — Make `loc_gate` ternary, not binary.** dliio's single decisive algorithmic win
  (Fisher p=0.0055; base 7/24 → 0–2/24 DIV; degeneracy never exceeded 1 axis). The
  binary on/off chatter near `loc_gate_trans_min_eig` is what *seeds* the runaway. Add a
  second, looser threshold and scale the projection by a linear partial-admit fraction in
  `armLocGate`/`applyLocGate` (~20 lines; reduces exactly to today's behaviour when the
  two thresholds are equal).
- **B2 — RESPLE's gate is translation-only; the tunnel failure is rotational (yaw).**
  `E_tt = Σ nnᵀ` captures only translation localizability; dliio eigendecomposes the
  rotation *and* translation Hessian blocks separately and found the tunnel fails on yaw
  (`FINDINGS_2026-06-22`). Extend the gate to the rotational / full pose information.
- **B3 — The gate FULLY removes LiDAR authority on the bad axis.** dliio: "zeroing the
  gain on a degenerate axis is catastrophic" (full freeze = 5/5 DIV); a *partial* admit
  is what works — which B1 provides for free.
- **B4 — Add a governor (output-pose clamp) as the fail-safe the gate lacks.** dliio:
  ~130× worst-case reduction (35 km → 270 m), n=24, sweet spot 0.15 m/scan (non-monotonic
  — 0.10 was *worse*). Caveat: RESPLE's state is the spline RCPs, so a clamp must fold
  back into the latest knot or live as an advisory output bound — needs spline-aware
  design. Pairs with the existing `loc_gate_cov_rate` inflation.
- **B5 — Robust kernel: data-driven (MAD) scale** instead of fixed `robust_delta`. RESPLE
  already *floors* rather than replaces the accept/reject gate (good); deriving the scale
  from `1.4826·MAD(zp)` per scan stops it silently pinning to L2 when residuals are small.
- **B6 — Do NOT chase isotropic auxiliary terms to stabilise the tunnel.** dliio's most
  expensive negative result: no isotropic term re-constrains a degenerate axis at any
  scalar weight (250×–10,000× shortfall); `GenZ pointWeight=100` was catastrophic. Bound
  the axis (B1–B4) instead.
- **B7 — Methodology** (already partly in the harness): n≥24 in chaotic basins; grep the
  built `.so` for a feature symbol before trusting an "ON" run; reproduce on a fresh build.

**Recommended sequence:** fix A1 (unlocks NIS recovery as a real fail-safe after A2) →
enable + ternary-ize `loc_gate` (B1) → extend to rotation (B2). A3/A4 are cheap
correctness fixes to land regardless.
