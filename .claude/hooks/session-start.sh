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
$SUDO apt-get update -qq || true
$SUDO apt-get install -y --no-install-recommends \
  build-essential cmake git curl gnupg ca-certificates lsb-release \
  libeigen3-dev libgtest-dev libomp-dev libboost-math-dev libyaml-cpp-dev

persist_ros_env() {
  if ! grep -q "/opt/ros/jazzy/setup.bash" "${CLAUDE_ENV_FILE:-/dev/null}" 2>/dev/null; then
    echo 'source /opt/ros/jazzy/setup.bash' >> "${CLAUDE_ENV_FILE}"
  fi
}

# --- ROS 2 Jazzy + PCL ------------------------------------------------------
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # Already provisioned (e.g. a custom base image from the repo Dockerfile).
  persist_ros_env
  echo "[session-start] ROS 2 Jazzy already present — toolchain ready."
elif curl -fsSL -o /dev/null https://packages.ros.org/ros2/ubuntu/dists/noble/InRelease 2>/dev/null; then
  # The ROS apt repo is reachable under this environment's network policy.
  if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    $SUDO curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi
  CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-noble}")"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
    | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  $SUDO apt-get update -qq
  $SUDO apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions \
    libpcl-dev ros-jazzy-pcl-conversions \
    ros-jazzy-tf2-ros ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-rclcpp-lifecycle ros-jazzy-rclcpp-action ros-jazzy-rclcpp-components \
    ros-jazzy-diagnostic-updater ros-jazzy-ament-cmake-gtest \
    ros-jazzy-launch-testing-ament-cmake ros-jazzy-rosbag2-storage-mcap
  persist_ros_env
  echo "[session-start] ROS 2 Jazzy + PCL installed — toolchain ready."
  echo "[session-start] NOTE: a full 'colcon build' of resple also needs the"
  echo "[session-start]       livox_ros_driver{,2}, livox_interfaces and"
  echo "[session-start]       estimate_msgs packages in the workspace src/."
else
  # The common sandbox case: the network policy blocks packages.ros.org (403).
  echo "[session-start] WARNING: packages.ros.org is blocked by this"
  echo "[session-start]   environment's network policy, so ROS 2 / PCL were"
  echo "[session-start]   NOT installed. The ROS-free unit tests still work"
  echo "[session-start]   (./scripts/run_unit_tests.sh). To enable the full"
  echo "[session-start]   PCL+ROS2 build/tests, either:"
  echo "[session-start]     1) broaden the environment network policy to allow"
  echo "[session-start]        packages.ros.org, or"
  echo "[session-start]     2) use a custom base image with ROS 2 Jazzy + PCL"
  echo "[session-start]        preinstalled (see the repo Dockerfile)."
fi
