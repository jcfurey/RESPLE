#!/usr/bin/env bash
# End-to-end accuracy smoke test for the RESPLE node (2026-07-11).
#
# Closes the "CI has no end-to-end estimator gate" gap WITHOUT needing a bag:
# the data_injector test harness publishes a deterministic stationary scene
# (a 10x10x4 m 6-plane room, seeded jitter) + a stationary IMU + identity
# static extrinsic TFs, so the ground truth is exact by construction — the
# pose must stay at the origin. The full pipeline runs for real (gravity
# init -> map init -> deskew -> parallel k-NN -> IEKF -> spline growth ->
# publishers), and the checker asserts:
#
#   * /odom actually publishes (>= MIN_ODOM_MSGS messages),
#   * the pose never leaves a MAX_DRIFT_M ball around the origin,
#   * it ENDS inside FINAL_DRIFT_M (converged, not oscillating out),
#   * the NIS detector never reports DIVERGED,
#   * the TF ownership guard never fires (clean single-owner setup).
#
# This is a coarse gate by design: it catches gross regressions anywhere in
# the pipeline (init, association, IEKF numerics, publishing) in ~1 minute,
# not fine accuracy drift — dataset APE/RPE comparisons still need real bags
# (scripts/overload_rehearsal.sh, run_sanitizer_replay.sh).
#
#   ./scripts/e2e_smoke.sh                # build + run + check (~3-10 min cold)
#   SECONDS_RUN=60 ./scripts/e2e_smoke.sh # longer soak
#   SKIP_BUILD=1 ./scripts/e2e_smoke.sh   # reuse an already-built workspace
set -euo pipefail

SECS="${SECONDS_RUN:-30}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${RESPLE_WS:-${HOME}/ros2_ws}"
TOOLS="${ROOT}/resple/test/tools"

[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy not found — run .claude/hooks/session-start.sh"; exit 1; }
set +u; source /opt/ros/jazzy/setup.bash; set -u

# The interpreter matters twice: rosidl generation needs NumPy, and the rclpy
# checker needs the SAME Python ABI Jazzy's C extensions were built for
# (3.12) — a stray /usr/local 3.11 "python3" imports Jazzy's site-packages
# but cannot load _rclpy_pybind11.
PYEXE="$(for p in /usr/bin/python3.12 /usr/bin/python3; do "$p" -c 'import numpy' 2>/dev/null >&2 && { echo "$p"; break; }; done)"

if [ "${SKIP_BUILD:-0}" != "1" ]; then
  mkdir -p "${WS}/src"
  for pkg in resple estimate_msgs AviaResple_msgs Mid70Avia_msgs HAP360_msgs; do
    ln -sfn "${ROOT}/${pkg}" "${WS}/src/${pkg}"
  done
  ln -sfn "${TOOLS}/data_injector" "${WS}/src/data_injector"
  cd "${WS}"
  echo "==> build resple (Release) + data_injector"
  colcon build --packages-up-to resple data_injector \
    --cmake-args -DENABLE_NATIVE_ARCH=OFF "-DPython3_EXECUTABLE=${PYEXE}"
else
  cd "${WS}"
fi
# shellcheck disable=SC1091
set +u; source install/setup.bash; set -u

LOGDIR="$(mktemp -d /tmp/resple_e2e_smoke.XXXX)"
echo "==> logs in ${LOGDIR}"

cleanup() {
  kill -INT "${RP:-}" "${INJ:-}" 2>/dev/null || true
  sleep 3
  kill -KILL "${RP:-}" "${INJ:-}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "==> run RESPLE + injector, checker collects for ${SECS}s"
./install/resple/lib/resple/RESPLE --ros-args \
  --params-file "${TOOLS}/inject_pointcloud2.yaml" > "${LOGDIR}/resple.log" 2>&1 &
RP=$!
sleep 2
./install/data_injector/lib/data_injector/injector > "${LOGDIR}/injector.log" 2>&1 &
INJ=$!

# The checker exits non-zero on any failed criterion.
if ! "${PYEXE}" "${ROOT}/scripts/e2e_smoke_check.py" --duration "${SECS}"; then
  echo "==> FAILED — last RESPLE log lines:"
  tail -30 "${LOGDIR}/resple.log" || true
  exit 1
fi
echo "==> e2e smoke PASSED"
