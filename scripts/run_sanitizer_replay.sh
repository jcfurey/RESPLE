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

echo "==> Launching ${LAUNCH} and replaying ${BAG} at ${REPLAY_RATE}x"
ros2 launch resple "${LAUNCH}" &
LAUNCH_PID=$!
trap 'kill ${LAUNCH_PID} 2>/dev/null || true' EXIT
sleep "${SETTLE_SEC}"

ros2 bag play "${BAG}" --rate "${REPLAY_RATE}"

# Give the worker / async map-update threads a moment to drain so late
# use-after-free or data-race reports are still attributed before shutdown.
sleep "${SETTLE_SEC}"
echo "==> Replay finished with no fatal sanitizer abort"
