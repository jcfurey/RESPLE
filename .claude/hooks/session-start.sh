#!/bin/bash
# SessionStart hook for Claude Code on the web.
#
# Installs the toolchain RESPLE needs so tests, linters and (most of) the
# colcon build work inside a web session: ROS 2 Jazzy + PCL + the package's
# build/test dependencies, plus the Eigen/GoogleTest/Boost the ROS-free unit
# tests use.  Idempotent and non-interactive; the container image is cached
# after the hook completes, so the heavy install only really runs once.
set -euo pipefail

# Only run in the remote sandbox; locally the developer manages their own env.
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

export DEBIAN_FRONTEND=noninteractive
SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

# The sandbox's network policy blocks Launchpad PPAs (deadsnakes, ondrej/php,
# ...), which makes every `apt-get update` fail with a 403 and would abort this
# hook under `set -e`. Those PPAs are irrelevant to RESPLE, so disable them.
grep -rlE "ppa\.launchpad" /etc/apt/sources.list.d/ 2>/dev/null \
  | xargs -r $SUDO rm -f || true
# We manage the ROS apt source ourselves and only (re)add it when the repo is
# actually reachable, so drop any stale copy first to avoid 403 update noise.
$SUDO rm -f /etc/apt/sources.list.d/ros2.list || true

# --- Base toolchain + ROS-free unit-test deps (always available) ------------
# These come from the Ubuntu archive, which the sandbox allows, so the ROS-free
# unit suite (scripts/run_unit_tests.sh) always works regardless of ROS access.
#   - libblas/liblapack: the package adds -DEIGEN_USE_BLAS globally, so any
#     target touching Eigen matrix products must LINK BLAS (see resple
#     CMakeLists test wiring). Present here so the full build links cleanly.
#   - python3-numpy + python3-dev: rosidl's Python message generator does
#     find_package(Python3 ... NumPy) and fails without the NumPy headers when
#     generating the in-repo message packages (estimate_msgs, the livox_* ones).
$SUDO apt-get update -qq || true
$SUDO apt-get install -y --no-install-recommends \
  build-essential cmake git curl gnupg ca-certificates lsb-release \
  libeigen3-dev libgtest-dev libomp-dev libboost-math-dev libyaml-cpp-dev \
  libblas-dev liblapack-dev python3-numpy python3-dev

persist_ros_env() {
  if ! grep -q "/opt/ros/jazzy/setup.bash" "${CLAUDE_ENV_FILE:-/dev/null}" 2>/dev/null; then
    echo 'source /opt/ros/jazzy/setup.bash' >> "${CLAUDE_ENV_FILE}"
  fi
}

# Probe packages.ros.org over BOTH https and http and echo the first scheme that
# answers. Some sandbox network policies 503/403 the TLS endpoint but allow plain
# http (observed: https -> 503, http -> 200). apt verifies the repo's GPG
# signature via the signed-by keyring regardless of transport, so an http source
# is safe here. Empty output => the repo is unreachable on either scheme.
ros_repo_scheme() {
  local scheme
  for scheme in https http; do
    if curl -fsSL -o /dev/null \
        "${scheme}://packages.ros.org/ros2/ubuntu/dists/noble/InRelease" 2>/dev/null; then
      echo "$scheme"; return 0
    fi
  done
  return 0
}

# --- ROS 2 Jazzy + PCL ------------------------------------------------------
ROS_SCHEME="$( [ -f /opt/ros/jazzy/setup.bash ] || ros_repo_scheme )"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # Already provisioned (e.g. a custom base image from the repo Dockerfile).
  persist_ros_env
  echo "[session-start] ROS 2 Jazzy already present — toolchain ready."
elif [ -n "${ROS_SCHEME}" ]; then
  # The ROS apt repo is reachable under this environment's network policy
  # (over ${ROS_SCHEME}).
  if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    $SUDO curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi
  CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-noble}")"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_SCHEME}://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
    | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  $SUDO apt-get update -qq
  # ros-dev-tools pulls colcon + the rosidl generators needed to build the
  # in-repo message packages.
  $SUDO apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions \
    libpcl-dev ros-jazzy-pcl-conversions \
    ros-jazzy-visualization-msgs ros-jazzy-nav-msgs ros-jazzy-std-srvs \
    ros-jazzy-tf2-ros ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-rclcpp-lifecycle ros-jazzy-rclcpp-action ros-jazzy-rclcpp-components \
    ros-jazzy-diagnostic-updater ros-jazzy-ament-cmake-gtest \
    ros-jazzy-launch-testing-ament-cmake ros-jazzy-rosbag2-storage-mcap
  persist_ros_env
  echo "[session-start] ROS 2 Jazzy + PCL installed (via ${ROS_SCHEME}) — toolchain ready."
  echo "[session-start] For a full package build/test, run: ./scripts/build_workspace.sh"
  echo "[session-start] (The livox_ros_driver{,2} / livox_interfaces and estimate_msgs"
  echo "[session-start]  packages it needs are already IN THIS REPO — the *_msgs dirs:"
  echo "[session-start]  AviaResple_msgs=livox_interfaces, Mid70Avia_msgs=livox_ros_driver,"
  echo "[session-start]  HAP360_msgs=livox_ros_driver2 — so no external clones are required.)"
else
  # The common sandbox case: the network policy blocks packages.ros.org on
  # both transports.
  echo "[session-start] WARNING: packages.ros.org is blocked by this"
  echo "[session-start]   environment's network policy (https and http), so"
  echo "[session-start]   ROS 2 / PCL were NOT installed. The ROS-free unit tests"
  echo "[session-start]   still work (./scripts/run_unit_tests.sh). To enable the"
  echo "[session-start]   full PCL+ROS2 build/tests, either:"
  echo "[session-start]     1) broaden the environment network policy to allow"
  echo "[session-start]        packages.ros.org, or"
  echo "[session-start]     2) use a custom base image with ROS 2 Jazzy + PCL"
  echo "[session-start]        preinstalled (see the repo Dockerfile)."
fi
