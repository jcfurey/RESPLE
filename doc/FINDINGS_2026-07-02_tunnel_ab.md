# 06042026 tunnel — degeneracy-lever A/B (2026-07-02)

Test of this cycle's degeneracy commits (X-ICP-style loc-gate `dc6afae`,
publish-side cov inflation `87f281f`, close-range down-weighting `5135d9d`,
narrow-tunnel template `19d9344`) on the 06042026 Ouster conduit, LIO. Harness +
raw data in the test-ws repo: `scripts/resple_tunnel_ab.sh`,
`results/resple_tunnel_ab/AB_results.csv`,
`results/SESSION_2026-07-02_resple_tunnel_ab.md`.

## Finding 0 (blocker) — `plane_min_cond_ratio: 0.05` freezes the pose at origin
The `config_narrow_tunnel.yaml` template ships `plane_min_cond_ratio: 0.05` (the
QR-pivot plane degeneracy guard, RESPLE.cpp ~1800). On this conduit that value
**rejects 100% of plane-fit correspondences** → `corresp_used: 0` in
`/resple_diagnostics` → the IEKF gets zero LiDAR constraint → **pose pinned at
(0,0,0) for the whole bag.** The guard was never bag-benchmarked ("0 = off,
pending bag benchmark" in CLAUDE.md); 0.05 is too aggressive here. **Use 0.0.**
Diagnosis tell: `corresp_passed_knn` high but `corresp_passed_plane: 0` = plane
stage is the rejecter (not loc_gate / NIS hold — those act after correspondences;
`recovery_hold_active`/`filter_state` were both inactive).

## Finding 1 — with the guard off, the degen levers do NOT help; they hurt
n=5 each, same code both arms, in-bag clipped:

| metric (median [min–max]) | baseline (levers off) | degen (guard 0.0 + loc_gate 0.02 + cov 0.05 + range_ref 1.0 + nis hold) |
|---|---|---|
| maxX excursion (m) | 11.2 [10.5–12.1] | 9.2 [7.7–16.1] |
| pathlen churn (m) | **183** [108–18055] | **34140** [5112–140503] |
| off-axis \|fy\|/\|fz\| max (m) | 0.9 / 0.0 | 23.8 / 8.9 |
| straight (net/path) | 0.10 | 0.00 |
| verdicts | 5× SLOSH | 3× SLOSH, 2× SHORT |

- **Neither config tracks** (`straight≈0` in all 10 reps): the tunnel axis is
  unobservable to LiDAR; no pure-LIO config makes net forward progress here.
- Where they differ, **degen is worse**: ~180× more path churn and large off-axis
  wander (23.8 m lateral vs 0.9 m). `nis hold` also gates `/odom` (fewer poses).
- Caveat: the in-bag clip removes post-bag IMU runaway — loc_gate's headline
  target. But baseline doesn't run away in-bag either (bounded ~12 m slosh), so
  this metric gives the gate no runaway-prevention credit. n=5 is underpowered for
  a p-value (chaotic basin, prior 06042026 work needed n≥24), but the
  off-axis/churn effect is large and consistent across all 5 degen reps.

## Takeaway
The along-axis degeneracy is fundamental; a downstream along-axis source (wheel
odom) or a visual modality is the real fix, not a LIO-only knob. Recommended
follow-ups: loc_gate value sweep (0.02 may be mistuned for this wider-than-narrow
bore) or isolate which lever hurts (nis hold vs the gate) — staged configs at
`results/config_06042026_gate{0005,001,005}.yaml`.
