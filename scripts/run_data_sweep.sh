#!/usr/bin/env bash
# Live data-path sanitizer sweep for RESPLE (HARDENING Phase 2.6).
#
# Drives the active node with synthetic LiDAR+IMU so the worker runs its full
# pipeline (deskew -> findCorresp parallel k-NN -> IEKF -> mapIncremental ->
# lasermapFovSegment -> spline growth), under ThreadSanitizer (default) or
# AddressSanitizer. The ROS Python CLI (ros2/ros2 bag/launch_test) is NOT
# required — a small C++ injector publishes static TF + IMU + PointCloud2, and
# RESPLE's main() auto-drives the lifecycle (configure->activate->spin).
#
#   ./scripts/run_data_sweep.sh            # TSan, num_threads=1 (real races)
#   ./scripts/run_data_sweep.sh asan       # ASan + UBSan
#   SECONDS_RUN=40 ./scripts/run_data_sweep.sh
#
# IMPORTANT — OpenMP false positives: with num_threads>1, TSan flags the IEKF's
# libgomp parallel-for barriers as ~hundreds of benign races (GCC libgomp is not
# TSan-annotated). This script forces num_threads=1 / OMP_NUM_THREADS=1 so the
# remaining reports are GENUINE cross-thread races. Re-run with num_threads=4 to
# measure parallel throughput, but interpret TSan output accordingly.
set -euo pipefail

MODE="${1:-tsan}"
SECS="${SECONDS_RUN:-25}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${RESPLE_WS:-${HOME}/ros2_ws}"
SUPP="${ROOT}/resple/test/tsan_suppressions.txt"
TOOLS="${ROOT}/resple/test/tools"

case "${MODE}" in
  tsan) SAN_ARG="-DENABLE_TSAN=ON" ;;
  asan) SAN_ARG="-DENABLE_ASAN=ON" ;;
  *) echo "usage: $0 [tsan|asan]"; exit 2 ;;
esac

[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy not found — run .claude/hooks/session-start.sh"; exit 1; }
set +u; source /opt/ros/jazzy/setup.bash; set -u

PYEXE="$(for p in /usr/bin/python3.12 /usr/bin/python3; do "$p" -c 'import numpy' 2>/dev/null && { echo "$p"; break; }; done)"

# Workspace: resple (sanitized) + the in-repo message packages + the injector.
mkdir -p "${WS}/src"
for pkg in resple estimate_msgs AviaResple_msgs Mid70Avia_msgs HAP360_msgs; do
  ln -sfn "${ROOT}/${pkg}" "${WS}/src/${pkg}"
done
ln -sfn "${TOOLS}/data_injector" "${WS}/src/data_injector"

cd "${WS}"
echo "==> build resple (${MODE}) + data_injector"
# --packages-select-cmake-args needs colcon-cmake >= 0.2.27; fall back to
# passing the sanitizer flag globally (the message packages and the injector
# simply ignore the unused ENABLE_* variable) on older colcon.
if colcon build --help 2>/dev/null | grep -q "packages-select-cmake-args"; then
  colcon build --packages-up-to resple data_injector \
    --cmake-args -DENABLE_NATIVE_ARCH=OFF "-DPython3_EXECUTABLE=${PYEXE}" \
    --packages-select-cmake-args resple ${SAN_ARG}
else
  colcon build --packages-up-to resple data_injector \
    --cmake-args -DENABLE_NATIVE_ARCH=OFF "-DPython3_EXECUTABLE=${PYEXE}" ${SAN_ARG}
fi
# shellcheck disable=SC1091
# (wrapped in +u like the ROS setup above: colcon's setup.bash reads
# COLCON_TRACE and trips `set -u` on some versions)
set +u; source install/setup.bash; set -u

# Config with num_threads=1 (see note above).
CFG="$(mktemp /tmp/inject_sweep.XXXX.yaml)"
sed 's/num_threads: 4/num_threads: 1/' "${TOOLS}/inject_pointcloud2.yaml" > "${CFG}"

LOGBASE="/tmp/resple_${MODE}_sweep"
rm -f "${LOGBASE}".*
if [ "${MODE}" = "tsan" ]; then
  export TSAN_OPTIONS="halt_on_error=0:history_size=4:suppressions=${SUPP}:log_path=${LOGBASE}"
else
  export ASAN_OPTIONS="detect_leaks=0:halt_on_error=0:log_path=${LOGBASE}"
  export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=0"
fi
export OMP_NUM_THREADS=1

echo "==> run RESPLE + injector for ${SECS}s"
./install/resple/lib/resple/RESPLE --ros-args --params-file "${CFG}" > /tmp/resple_sweep.log 2>&1 &
RP=$!
sleep 2
./install/data_injector/lib/data_injector/injector > /tmp/injector_sweep.log 2>&1 &
INJ=$!
sleep "${SECS}"
kill -INT "${RP}" "${INJ}" 2>/dev/null || true
sleep 4
kill -KILL "${RP}" "${INJ}" 2>/dev/null || true
wait 2>/dev/null || true

echo "==> steady state:"; grep -iE "Gravity alignment successful|initialization complete|heartbeat" /tmp/resple_sweep.log | tail -3 || true
echo "==> sanitizer reports (${LOGBASE}.*):"
if ls "${LOGBASE}".* >/dev/null 2>&1; then
  grep -h "WARNING: ThreadSanitizer\|ERROR: AddressSanitizer\|runtime error:" "${LOGBASE}".* | sort | uniq -c
  echo "    (full reports in ${LOGBASE}.<pid>)"
else
  echo "    none — clean."
fi
