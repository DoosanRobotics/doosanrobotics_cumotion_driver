#!/usr/bin/env bash
set -euo pipefail

echo "[build-doosan] === START Combined Build Process ==="

ISAAC_WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
DOOSAN_WS="${DOOSAN_WS:-/workspaces/ros2_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

ISAAC_SRC="${ISAAC_WS}/src"
DOOSAN_SRC="${DOOSAN_WS}/src"

echo "[build-doosan] Isaac workspace  : ${ISAAC_WS}"
echo "[build-doosan] Doosan workspace : ${DOOSAN_WS}"
echo "[build-doosan] ROS_DISTRO       : ${ROS_DISTRO}"
echo "-------------------------------------------------------------"

# 1. 기존 build/install/log 제거
# echo "[build-doosan] Cleaning previous builds..."
# rm -rf ${ISAAC_WS}/build ${ISAAC_WS}/install ${ISAAC_WS}/log || true
# rm -rf ${DOOSAN_WS}/build ${DOOSAN_WS}/install ${DOOSAN_WS}/log || true
# mkdir -p ${LOG_DIR}

# 1. ROS 환경 로드
sudo apt update
# 2. Isaac Workspace Build
echo "[build-doosan] === Building Isaac ROS Workspace ==="

if [ -d "${ISAAC_SRC}" ]; then
  cd "${ISAAC_WS}"

  rosdep update || true
  rosdep install -i -r \
    --from-paths "${ISAAC_SRC}" \
    --rosdistro ${ROS_DISTRO} --ignore-src -y || true

  colcon build \
    --merge-install \
    --parallel-workers 8
else
  echo "[build-doosan] Isaac src not found — skipping."
fi

echo "-------------------------------------------------------------"

# 3. Doosan Workspace Build
echo "[build-doosan] === Building Doosan Workspace ==="

if [ -d "${DOOSAN_SRC}" ]; then
  cd "${DOOSAN_WS}"

  rosdep update || true
  rosdep install -i -r \
    --from-paths "${DOOSAN_SRC}" \
    --rosdistro ${ROS_DISTRO} --ignore-src -y || true

  colcon build \
    --merge-install \
    --parallel-workers 8

else
  echo "[build-doosan] Doosan src not found — skipping."
fi

echo "-------------------------------------------------------------"
echo "[build-doosan] COMPLETE."
