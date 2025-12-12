#!/usr/bin/env bash
set -eo pipefail

# Isaac ROS + Doosan Combined Build Script

ISAAC_WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
DOOSAN_WS="${DOOSAN_WS:-/workspaces/ros2_ws}"

ISAAC_SRC="${ISAAC_WS}/src"
DOOSAN_SRC="${DOOSAN_WS}/src"

echo "===================================================="
echo "[build-doosan] Isaac WS : ${ISAAC_WS}"
echo "[build-doosan] Doosan WS: ${DOOSAN_WS}"
echo "===================================================="

# ROS Base Environment
echo "[build-doosan] === Sourcing base ROS environment (${ROS_DISTRO}) ==="

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "[ERROR] ROS environment not found at /opt/ros/${ROS_DISTRO}"
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# System Dependencies
echo "[build-doosan] Checking essential dependencies..."

sudo apt-get update --allow-releaseinfo-change -qq || true

sudo apt-get install -y --no-install-recommends \
  libpoco-dev \
  libyaml-cpp-dev \
  wget \
  dbus-x11 \
  ros-${ROS_DISTRO}-control-msgs \
  ros-${ROS_DISTRO}-realtime-tools \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-gazebo-msgs \
  ros-${ROS_DISTRO}-moveit-msgs \
  ros-${ROS_DISTRO}-moveit-configs-utils \
  ros-${ROS_DISTRO}-moveit-ros-move-group \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ign-ros2-control || true

 # Isaac ROS Workspace Build / Overlay
echo ""
echo "[build-doosan] === Building Isaac ROS workspace (${ISAAC_WS}) ==="

if [[ -f "${ISAAC_WS}/install/setup.bash" ]]; then
  echo "[build-doosan] Isaac workspace already built — overlaying."
  rosdep update || true
  rosdep install -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y
  source "${ISAAC_WS}/install/setup.bash"

elif [[ -d "${ISAAC_SRC}" ]]; then
  echo "[build-doosan] Isaac install not found — building from source."
  cd "${ISAAC_WS}"

  rosdep update || true
  rosdep install -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y

  colcon build
  source "${ISAAC_WS}/install/setup.bash"

  echo "[build-doosan] Isaac workspace built successfully."
else
  echo "[WARN] Isaac src directory not found — skipping Isaac build."
fi

# Doosan Workspace Build / Emulator Installation
echo ""
echo "[build-doosan] === Building Doosan workspace (${DOOSAN_WS}) ==="

if [[ ! -d "${DOOSAN_SRC}" ]]; then
  echo "[WARN] No ${DOOSAN_SRC} directory found — skipping Doosan build."
else

  # Optional emulator install
  if [[ -d "${DOOSAN_SRC}/doosan-robot2" ]] && \
     [[ -f "${DOOSAN_SRC}/doosan-robot2/install_emulator.sh" ]]; then

    echo "[build-doosan] Installing Doosan emulator..."
    cd "${DOOSAN_SRC}/doosan-robot2"
    chmod +x ./install_emulator.sh
    sudo ./install_emulator.sh || echo "[WARN] Emulator installation failed — continuing..."

  fi

  cd "${DOOSAN_WS}"

  if [[ -f "${DOOSAN_WS}/install/setup.bash" ]]; then
    echo "[build-doosan] Doosan install already present — skipping colcon build."
  else
    echo "[build-doosan] Building Doosan workspace..."
    colcon build || echo "[WARN] colcon build failed for some packages."
  fi
  rosdep update || true
  rosdep install -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y
  source "${DOOSAN_WS}/install/setup.bash"
fi

# GPU / DCGM Group Permission (Optional)
if getent group nvidia-dcgm >/dev/null; then
  echo "[build-doosan] Adding admin to nvidia-dcgm group..."
  sudo usermod -aG nvidia-dcgm admin || true
else
  echo "[build-doosan] Group 'nvidia-dcgm' not found — skipping."
fi

# Docker Socket Permission
if [[ -S /var/run/docker.sock ]]; then
  echo "[build-doosan] Setting docker.sock permission (666)"
  sudo chmod 666 /var/run/docker.sock || true
fi

echo ""
echo "[build-doosan] Build complete!"
echo "-----------------------------------------------------------"
cd "${DOOSAN_WS}"
