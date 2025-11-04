#!/usr/bin/env bash
set -eo pipefail

# Isaac ROS Workspace Configuration Script for Doosan Setup

# Default Isaac ROS workspace path (can be overridden)
ISAAC_ROS_WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "[startup] Script directory detected: ${SCRIPT_DIR}"

# Target directories inside the Isaac ROS workspace
TARGET_DOCKER_DIR="${ISAAC_ROS_WS}/src/isaac_ros_common/docker"
TARGET_SCRIPT_DIR="${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts"
TARGET_CONFIG_DIR="${ISAAC_ROS_WS}/src/isaac_ros_common/scripts"

# Ensure that target directories exist
mkdir -p "${TARGET_DOCKER_DIR}" "${TARGET_SCRIPT_DIR}" "${TARGET_CONFIG_DIR}"

echo "[startup] Copying Doosan Docker and configuration files..."

# Copy Dockerfile
if [ -f "${SCRIPT_DIR}/Dockerfile.doosan" ]; then
  cp "${SCRIPT_DIR}/Dockerfile.doosan" "${TARGET_DOCKER_DIR}/"
  echo "[OK] Copied Dockerfile.doosan"
else
  echo "[WARN] Dockerfile.doosan not found in ${SCRIPT_DIR}"
fi

# Copy build-doosan.sh
if [ -f "${SCRIPT_DIR}/build-doosan.sh" ] || [ -f "${SCRIPT_DIR}/build_doosan.sh" ]; then
  cp "${SCRIPT_DIR}"/build-doosan.sh* "${TARGET_SCRIPT_DIR}/" 2>/dev/null || true
  echo "[OK] Copied build-doosan.sh"
else
  echo "[WARN] build-doosan.sh not found in ${SCRIPT_DIR}"
fi

# Copy .isaac_ros_common-config
if [ -f "${SCRIPT_DIR}/.isaac_ros_common-config" ]; then
  cp "${SCRIPT_DIR}/.isaac_ros_common-config" "${TARGET_CONFIG_DIR}/"
  echo "[OK] Copied .isaac_ros_common-config"
else
  echo "[WARN] .isaac_ros_common-config not found, creating default..."
  echo "CONFIG_IMAGE_KEY=ros2_humble.doosan" > "${TARGET_CONFIG_DIR}/.isaac_ros_common-config"
fi

# Copy .isaac_ros_dev-dockerargs
if [ -f "${SCRIPT_DIR}/.isaac_ros_dev-dockerargs" ]; then
  cp "${SCRIPT_DIR}/.isaac_ros_dev-dockerargs" "${TARGET_CONFIG_DIR}/"
  echo "[OK] Copied .isaac_ros_dev-dockerargs"
else
  echo "[WARN] .isaac_ros_dev-dockerargs not found, creating default..."
  echo "CONFIG_IMAGE_KEY=ros2_humble.doosan" > "${TARGET_CONFIG_DIR}/.isaac_ros_dev-dockerargs"
fi

# Completion message
echo "[startup] Doosan configuration and Dockerfile successfully synchronized."
echo "[startup] You can now run the development container using:"
echo "    cd ${ISAAC_ROS_WS}/src/isaac_ros_common"
echo "    ./run_dev.sh"
