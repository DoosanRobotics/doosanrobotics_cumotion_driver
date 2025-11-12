#!/usr/bin/env bash
set -eo pipefail

# Isaac ROS + Doosan Combined Build Script
ISAAC_WS=${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}
DOOSAN_WS=${DOOSAN_WS:-/ros2_ws}

ISAAC_SRC="${ISAAC_WS}/src"
DOOSAN_SRC="${DOOSAN_WS}/src"

echo "[build-doosan] === Sourcing base ROS environment (${ROS_DISTRO}) ==="
if ! source "/opt/ros/${ROS_DISTRO}/setup.bash"; then
  echo "[ERROR] ROS environment not found at /opt/ros/${ROS_DISTRO}"
  exit 1
fi

# Install essential dependencies (optional)
echo "[build-doosan] Checking essential dependencies..."
sudo apt-get update --allow-releaseinfo-change --allow-releaseinfo-change-origin --allow-releaseinfo-change-label -qq || true-qq
sudo apt-get install -y --no-install-recommends \
  libpoco-dev libyaml-cpp-dev wget dbus-x11 \
  ros-${ROS_DISTRO}-control-msgs ros-${ROS_DISTRO}-realtime-tools \
  ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-moveit-msgs \
  ros-${ROS_DISTRO}-moveit-configs-utils ros-${ROS_DISTRO}-moveit-ros-move-group \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ign-ros2-control || true

# Isaac ROS Workspace Build / Overlay
echo "[build-doosan] === Building Isaac ROS workspace (${ISAAC_WS}) ==="

if [ -f "${ISAAC_WS}/install/setup.bash" ]; then
  echo "[build-doosan] Isaac workspace already built — overlaying."
  rosdep update || true
  rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
  source "${ISAAC_WS}/install/setup.bash"

elif [ -d "${ISAAC_SRC}" ]; then
  echo "[build-doosan] Isaac install not found — building from source."
  cd "${ISAAC_WS}"

  rosdep update || true
  rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

  colcon build --packages-skip nvblox_test_data nvblox_test
  source "${ISAAC_WS}/install/setup.bash"
  echo "[build-doosan] Isaac workspace built successfully."
else
  echo "[build-doosan] ⚠️  Isaac src directory not found — skipping Isaac build."
fi

# Doosan Workspace Build / Emulator Installation
echo "[build-doosan] === Building Doosan workspace (${DOOSAN_WS}) ==="

if [ ! -d "${DOOSAN_SRC}" ]; then
  echo "[build-doosan] ❌ No ${DOOSAN_SRC} directory found — skipping Doosan build."
else
  # Optional emulator install if present
  if [ -d "${DOOSAN_SRC}/doosan-robot2" ] && [ -f "${DOOSAN_SRC}/doosan-robot2/install_emulator.sh" ]; then
    echo "[build-doosan] Installing Doosan emulator..."
    cd "${DOOSAN_SRC}/doosan-robot2"
    chmod +x ./install_emulator.sh
    sudo ./install_emulator.sh || echo "[WARN] Emulator installation failed — continuing..."
  fi

  cd "${DOOSAN_WS}"
  if [ -f "${DOOSAN_WS}/install/setup.bash" ]; then
    echo "[build-doosan] Doosan install already present — skipping build."
    rosdep update || true
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
  else
    rosdep update || true
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

    colcon build || {
      echo "[WARN] colcon build failed for some packages — continuing..."
    }
    echo "[build-doosan] Doosan workspace built successfully."
  fi
fi

# Add Group Permission (for GPU/DCGM monitoring)
if getent group nvidia-dcgm >/dev/null; then
  echo "[build-doosan] Adding admin to nvidia-dcgm group..."
  sudo usermod -aG nvidia-dcgm admin || true
else
  echo "[build-doosan]   Group 'nvidia-dcgm' not found — skipping."
fi

sudo chmod 666 /var/run/docker.sock

# Summary
echo ""
echo "[build-doosan] Build complete!"
echo "-----------------------------------------------------------"
cd "${DOOSAN_WS}"
