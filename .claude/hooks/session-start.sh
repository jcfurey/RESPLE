#!/bin/bash
# SessionStart hook: provision a ROS2 Jazzy build environment for RESPLE
# in Claude Code on the web sandboxes (Ubuntu 24.04).
set -euo pipefail

# Only run in remote (Claude Code on the web) environments
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

export DEBIAN_FRONTEND=noninteractive
ROS_DISTRO=jazzy

# --- 1. ROS2 apt repository (idempotent) ---
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  apt-get update -qq
  apt-get install -y -qq curl gnupg lsb-release
  curl -sSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update -qq
fi

# --- 2. ROS2 + build dependencies (idempotent; no-op when already installed) ---
apt-get install -y -qq \
  ros-${ROS_DISTRO}-ros-base \
  ros-${ROS_DISTRO}-pcl-conversions \
  ros-${ROS_DISTRO}-visualization-msgs \
  ros-${ROS_DISTRO}-std-srvs \
  ros-${ROS_DISTRO}-ament-lint-auto \
  ros-${ROS_DISTRO}-ament-lint-common \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-dev \
  python3-numpy \
  libpcl-dev \
  libeigen3-dev \
  libomp-dev \
  libyaml-cpp-dev

# --- 3. ROS2 workspace with this repo as a source package ---
WS="$HOME/ros2_ws"
mkdir -p "$WS/src"
ln -sfn "$CLAUDE_PROJECT_DIR" "$WS/src/RESPLE"

# --- 4. Build the workspace (matches README compilation instructions) ---
# ROS setup scripts reference unset variables, so relax -u while sourcing
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
set -u
cd "$WS"
# The sandbox's default python3 may not match the interpreter ROS Jazzy
# targets (3.12 on Ubuntu 24.04), so pin it explicitly for rosidl
PYTHON3_BIN=$(ls /opt/ros/${ROS_DISTRO}/lib/ | grep -o 'python3\.[0-9]*' | head -1)
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/${PYTHON3_BIN} \
  --packages-select estimate_msgs livox_ros_driver livox_interfaces livox_ros_driver2 resple

# --- 5. Make the environment available to the session ---
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  {
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "source $WS/install/setup.bash"
  } >> "$CLAUDE_ENV_FILE"
fi

echo "ROS2 ${ROS_DISTRO} build environment ready (workspace: $WS)"
