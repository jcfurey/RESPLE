#!/usr/bin/env bash
# Bag-replay smoke test under AddressSanitizer or ThreadSanitizer.
#
# This is the verification leg for the concurrency / memory-safety hardening:
# build the package with a sanitizer, replay a recorded bag through the RESPLE
# + Mapping lifecycle nodes, and fail if the sanitizer reports anything. It
# REQUIRES a working ROS 2 (Jazzy) workspace — it does not run in a bare
# container — and is intended for a developer machine or CI runner with the
# datasets available.
#
# Usage:
#   ./scripts/run_sanitizer_replay.sh <asan|tsan> <path/to/bag> [launch_file]
#
# Example:
#   ./scripts/run_sanitizer_replay.sh tsan ~/bags/ouster_short \
#       resple_ouster.launch.py
set -euo pipefail

MODE="${1:-asan}"
BAG="${2:?usage: run_sanitizer_replay.sh <asan|tsan> <bag> [launch_file]}"
LAUNCH="${3:-resple_ouster.launch.py}"
REPLAY_RATE="${REPLAY_RATE:-1.0}"
SETTLE_SEC="${SETTLE_SEC:-5}"

case "${MODE}" in
  asan) CMAKE_SAN="-DENABLE_ASAN=ON" ;;
  tsan) CMAKE_SAN="-DENABLE_TSAN=ON" ;;
  *) echo "MODE must be 'asan' or 'tsan'"; exit 2 ;;
esac

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 2 not sourced (ROS_DISTRO empty). Source /opt/ros/<distro>/setup.bash first." >&2
  exit 3
fi

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS}"

echo "==> Building resple with ${MODE} (${CMAKE_SAN})"
colcon build --packages-up-to resple \
  --cmake-args "${CMAKE_SAN}" -DENABLE_DEBUG_O1=ON --event-handlers console_direct+
# shellcheck disable=SC1091
source install/setup.bash

# Surface, rather than swallow, sanitizer findings; abort on the first error.
export ASAN_OPTIONS="halt_on_error=1:abort_on_error=1:detect_leaks=1:${ASAN_OPTIONS:-}"
export TSAN_OPTIONS="halt_on_error=1:second_deadlock_stack=1:${TSAN_OPTIONS:-}"

# --- Where the sanitizer output actually lands -------------------------------
# WHY this matters for the gate: every launch file in resple/launch/ runs the
# nodes with output='log' (verified: resple_ouster.launch.py:78,91), so the
# nodes' stdout/stderr — and therefore every ASan/TSan report, which the
# runtime writes to *stderr* — never reaches this script's terminal or a
# redirect of `ros2 launch`. It goes to the launch log directory. Pin that
# directory so the post-run scan below is deterministic, and still sweep
# ~/.ros/log for anything that ignored the override (a nested launch, a node
# that re-execs, an older rcl).
RUN_DIR="$(mktemp -d /tmp/resple_san_replay.XXXX)"
export ROS_LOG_DIR="${RUN_DIR}/ros_log"
mkdir -p "${ROS_LOG_DIR}"
LAUNCH_LOG="${RUN_DIR}/launch.log"
# Marker file: restrict the ~/.ros/log sweep to files touched by THIS run so a
# stale report from an earlier replay cannot fail today's.
STAMP="${RUN_DIR}/.start_stamp"
touch "${STAMP}"
echo "==> Run logs: ${LAUNCH_LOG}"
echo "    Node logs (output='log' — where sanitizer reports go): ${ROS_LOG_DIR}"

echo "==> Launching ${LAUNCH} and replaying ${BAG} at ${REPLAY_RATE}x"
ros2 launch resple "${LAUNCH}" > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
trap 'kill ${LAUNCH_PID} 2>/dev/null || true' EXIT
sleep "${SETTLE_SEC}"

# WHY: nothing downstream ever looked at the launch process, so the script's
# status depended solely on `ros2 bag play` — a node that aborted on a
# sanitizer report during startup left the replay publishing into the void and
# the script exiting 0. Check before spending the replay time.
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  LAUNCH_RC=0
  wait "${LAUNCH_PID}" || LAUNCH_RC=$?
  echo "ERROR: launch died during the ${SETTLE_SEC}s settle window (exit ${LAUNCH_RC})." >&2
  tail -40 "${LAUNCH_LOG}" >&2 || true
  exit 4
fi

BAG_RC=0
ros2 bag play "${BAG}" --rate "${REPLAY_RATE}" || BAG_RC=$?

# Give the worker / async map-update threads a moment to drain so late
# use-after-free or data-race reports are still attributed before shutdown.
sleep "${SETTLE_SEC}"

# --- Gates -------------------------------------------------------------------
# 1. Did the sanitized node survive the whole replay? ASAN_OPTIONS/TSAN_OPTIONS
#    above set abort/halt_on_error, so a finding kills the node — which used to
#    be completely invisible here.
NODE_DIED=0
LAUNCH_RC=0
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  NODE_DIED=1
  wait "${LAUNCH_PID}" || LAUNCH_RC=$?
  echo "ERROR: launch exited before the replay finished (exit ${LAUNCH_RC})." >&2
fi

# 2. Any sanitizer report in the launch log or the node logs? Scan by content,
#    not by exit status: with halt_on_error=0 (if the caller overrides it) or a
#    report on a non-fatal path the node stays up and only the log shows it.
SAN_RE='ERROR: AddressSanitizer|WARNING: ThreadSanitizer|ERROR: LeakSanitizer|SUMMARY: (Address|Thread|Undefined|Leak)Sanitizer|runtime error:'
mapfile -t SCAN_FILES < <(
  { printf '%s\n' "${LAUNCH_LOG}"
    find "${ROS_LOG_DIR}" "${HOME}/.ros/log" -type f -newer "${STAMP}" 2>/dev/null
  } | sort -u
)
SAN_FILES=""
if [ "${#SCAN_FILES[@]}" -gt 0 ]; then
  SAN_FILES="$(grep -lE "${SAN_RE}" "${SCAN_FILES[@]}" 2>/dev/null || true)"
fi
if [ -n "${SAN_FILES}" ]; then
  echo "ERROR: ${MODE} reported findings in:" >&2
  printf '%s\n' "${SAN_FILES}" | while read -r f; do
    echo "  --- ${f}" >&2
    grep -nE "${SAN_RE}" "${f}" | head -20 >&2 || true
  done
fi

if [ "${NODE_DIED}" -eq 1 ] || [ -n "${SAN_FILES}" ] || [ "${BAG_RC}" -ne 0 ]; then
  echo "==> FAILED: node_died=${NODE_DIED} (exit ${LAUNCH_RC}), bag_play=${BAG_RC}," \
       "sanitizer_reports=$([ -n "${SAN_FILES}" ] && echo yes || echo no)." >&2
  echo "    Full logs: ${LAUNCH_LOG} and ${ROS_LOG_DIR}" >&2
  exit 1
fi

echo "==> Replay finished: node alive, bag replay clean, no ${MODE} reports"
echo "    (scanned ${#SCAN_FILES[@]} log file(s) under ${ROS_LOG_DIR} + ${LAUNCH_LOG})"
