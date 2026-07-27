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
  # ENABLE_TSAN/ASAN pinned OFF explicitly. CMake CACHES those options, so
  # running this script after scripts/run_data_sweep.sh (or any manual
  # sanitizer build) in the same workspace silently produced a SANITIZER build
  # here — and the smoke test then failed on "odom messages >= 200" (measured
  # 21 instead of ~3000, under TSan's ~30x slowdown), which reads exactly like
  # an estimator regression. Pin them so the verdict means what it says.
  colcon build --packages-up-to resple data_injector \
    --cmake-args -DENABLE_NATIVE_ARCH=OFF -DENABLE_TSAN=OFF -DENABLE_ASAN=OFF \
                 "-DPython3_EXECUTABLE=${PYEXE}"
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

# WHY a liveness helper at all: every criterion the checker evaluates is of the
# form "enough messages were banked / the pose never left the ball", which a
# node that CRASHED late in the collection window still satisfies from the data
# it published while it was healthy. Process liveness is a PRECONDITION for the
# verdict, not a detail — without it a segfault at t=25s of a 30s run passes.
#
# kill -0 is the probe; the /proc State check is the necessary companion. bash
# normally reaps an exited background child asynchronously (so its PID is gone
# and kill -0 fails, as intended), but in the window before that reap the child
# is a ZOMBIE which still has a /proc entry and on which kill -0 SUCCEEDS —
# reporting a dead node as healthy, the exact bug this gate is closing.
alive() {
  local pid="${1:-}"
  [ -n "${pid}" ] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  ! grep -q '^State:[[:space:]]*Z' "/proc/${pid}/status" 2>/dev/null
}

# Exit status of an already-exited child (bash keeps it until waited on).
child_status() {
  local rc=0
  wait "${1}" 2>/dev/null || rc=$?
  echo "${rc}"
}

echo "==> run RESPLE + injector, checker collects for ${SECS}s"
# Optional extra node params, for exercising a default-off feature against the
# known-ground-truth scene without editing the params file:
#   RESPLE_EXTRA_ARGS="-p lidar_gate_sigma:=5.0" ./scripts/e2e_smoke.sh
# Empty by default — the pass criteria below are calibrated for the shipped
# defaults, so a run with overrides proves "this path still works", not "this
# path is better".
read -r -a EXTRA_ARGS <<< "${RESPLE_EXTRA_ARGS:-}"
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  echo "==> extra params: ${EXTRA_ARGS[*]}"
fi
./install/resple/lib/resple/RESPLE --ros-args \
  --params-file "${TOOLS}/inject_pointcloud2.yaml" \
  "${EXTRA_ARGS[@]}" > "${LOGDIR}/resple.log" 2>&1 &
RP=$!
sleep 2
# Fail fast on a startup abort (bad params, missing plugin, SIGILL from a
# -march mismatch) instead of spending the whole window collecting nothing.
if ! alive "${RP}"; then
  echo "==> FAILED: RESPLE exited during startup with status $(child_status "${RP}")" >&2
  tail -30 "${LOGDIR}/resple.log" >&2 || true
  exit 1
fi
./install/data_injector/lib/data_injector/injector > "${LOGDIR}/injector.log" 2>&1 &
INJ=$!

# The checker exits non-zero on any failed criterion. Capture rather than
# short-circuit, so the liveness gate below is evaluated FIRST — a crash is a
# more specific and more actionable diagnosis than "criterion X not met".
CHECK_RC=0
"${PYEXE}" "${ROOT}/scripts/e2e_smoke_check.py" --duration "${SECS}" || CHECK_RC=$?

DIED=0
if ! alive "${RP}"; then
  echo "==> FAILED: RESPLE died during the ${SECS}s run — exit status $(child_status "${RP}")" >&2
  echo "    (139/SIGSEGV=11, 134/SIGABRT=6; last RESPLE log lines:)" >&2
  tail -30 "${LOGDIR}/resple.log" >&2 || true
  DIED=1
fi
if ! alive "${INJ}"; then
  echo "==> FAILED: data_injector died during the ${SECS}s run — exit status $(child_status "${INJ}")" >&2
  echo "    (the scene stopped being published, so any criterion it fed is void)" >&2
  tail -30 "${LOGDIR}/injector.log" >&2 || true
  DIED=1
fi
if [ "${DIED}" -ne 0 ]; then
  echo "==> e2e smoke FAILED (process exited early; checker verdict was ${CHECK_RC})" >&2
  echo "    logs: ${LOGDIR}" >&2
  exit 1
fi

if [ "${CHECK_RC}" -ne 0 ]; then
  echo "==> FAILED (checker exit ${CHECK_RC}) — last RESPLE log lines:"
  tail -30 "${LOGDIR}/resple.log" || true
  exit 1
fi
echo "==> e2e smoke PASSED (RESPLE + injector both still alive at the end)"
