#!/usr/bin/env bash
# Full colcon build of the resple package against the real ROS 2 / PCL stack.
#
# This encodes the workarounds discovered while bringing the full build up in a
# Claude-Code-on-the-web session, so the next session can do it in one command:
#
#   ./scripts/build_workspace.sh           # build up to resple
#   ./scripts/build_workspace.sh --test    # build, then run the package tests
#
# Prereqs: the SessionStart hook (.claude/hooks/session-start.sh) installs the
# toolchain — ROS 2 Jazzy + PCL + colcon + rosidl generators + NumPy + BLAS. If
# ROS is not under /opt/ros/jazzy this script bails with a pointer to the hook.
#
# Things this handles that a bare `colcon build` does not:
#   1. The Livox message packages RESPLE depends on are ALREADY IN THIS REPO,
#      just under different directory names:
#         AviaResple_msgs -> package livox_interfaces
#         Mid70Avia_msgs  -> package livox_ros_driver
#         HAP360_msgs     -> package livox_ros_driver2
#      plus estimate_msgs. We symlink them (and resple) into a clean workspace
#      src/ so colcon finds them — no external clones needed.
#   2. The image ships several Python interpreters and /usr/local/bin/python3
#      may be a 3.11 with NO working NumPy, which CMake's FindPython3 would pick
#      for rosidl's message generator and then fail. We pick an interpreter that
#      can actually import NumPy (preferring the ROS system python3.12) and pass
#      it as -DPython3_EXECUTABLE.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${RESPLE_WS:-${HOME}/ros2_ws}"
RUN_TESTS=0
[ "${1:-}" = "--test" ] && RUN_TESTS=1

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy." >&2
  echo "Run the SessionStart hook first (or broaden the network policy):" >&2
  echo "  CLAUDE_CODE_REMOTE=true .claude/hooks/session-start.sh" >&2
  exit 1
fi
# The ROS setup scripts reference unbound vars (AMENT_TRACE_SETUP_FILES, ...),
# so relax `set -u` just for the source.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

# --- Pick a Python interpreter whose NumPy actually imports -----------------
pick_python() {
  local py
  for py in /usr/bin/python3.12 /usr/bin/python3.11 /usr/bin/python3 python3; do
    command -v "$py" >/dev/null 2>&1 || continue
    if "$py" -c 'import numpy' >/dev/null 2>&1; then
      command -v "$py" >/dev/null 2>&1 && readlink -f "$(command -v "$py")"
      return 0
    fi
  done
  return 1
}
PYEXE="$(pick_python || true)"
if [ -z "${PYEXE}" ]; then
  echo "ERROR: no Python interpreter with a working NumPy found." >&2
  echo "Install it, e.g.: sudo apt-get install -y python3-numpy python3-dev" >&2
  exit 1
fi
echo "==> Using Python: ${PYEXE} ($(${PYEXE} --version 2>&1))"

# --- Assemble a clean workspace from the in-repo packages -------------------
echo "==> Workspace: ${WS}"
mkdir -p "${WS}/src"
# Drop only the symlinks we manage; leave any real user content alone.
find "${WS}/src" -maxdepth 1 -type l -delete 2>/dev/null || true
for pkg in resple estimate_msgs AviaResple_msgs Mid70Avia_msgs HAP360_msgs; do
  if [ -d "${ROOT}/${pkg}" ]; then
    ln -sfn "${ROOT}/${pkg}" "${WS}/src/${pkg}"
  fi
done
echo "    linked: $(ls "${WS}/src")"

# --- Build ------------------------------------------------------------------
cd "${WS}"
echo "==> colcon build --packages-up-to resple"
colcon build --packages-up-to resple \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DENABLE_NATIVE_ARCH=OFF \
    "-DPython3_EXECUTABLE=${PYEXE}"

if [ "${RUN_TESTS}" -eq 1 ]; then
  echo "==> colcon test --packages-select resple"
  # WHY the flag and the missing `|| true`: `colcon test` on its own exits 0
  # even when tests FAIL — its status reflects only build/infrastructure errors,
  # not test verdicts — and the old `colcon test-result ... || true` then threw
  # away the one status that did carry the verdict. Net effect: `--test` ALWAYS
  # exited 0, so no caller (CI, a wrapper script, a human `&&` chain) could ever
  # notice a red test. Capture both statuses so the per-test summary (which names
  # the failing test) still prints, then propagate.
  TEST_RC=0
  colcon test --packages-select resple --return-code-on-test-failure || TEST_RC=$?
  RESULT_RC=0
  colcon test-result --test-result-base build/resple --all || RESULT_RC=$?
  if [ "${TEST_RC}" -ne 0 ] || [ "${RESULT_RC}" -ne 0 ]; then
    echo "ERROR: resple tests FAILED (colcon test=${TEST_RC}, test-result=${RESULT_RC})." >&2
    exit 1
  fi
fi

echo "==> Done. Source the overlay with:  source ${WS}/install/setup.bash"
