# Isaac ROS + Doosan ROS2 Integration Setup Guide

This guide explains how to build and configure an integrated environment combining **NVIDIA Isaac ROS (release 3.2)** and **Doosan ROS 2** for GPU-accelerated motion planning using **cuMotion** and **MoveIt 2**.

---

## 1. System Environment

- Isaac Sim: 4.2.0  
- Isaac ROS: release 3.2  
- NVIDIA Driver: 570 (recommended)  
- CUDA: 12.8 (Docker image base 12.6)  
- ROS 2: Humble (compatible with Isaac ROS 3.2)

---

## 2. System Preparation

### 2.1 Locale Configuration

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

### 2.2 Base Packages and Repository Setup

```bash
sudo apt update && sudo apt install -y curl gnupg software-properties-common
sudo add-apt-repository universe
sudo apt-get update

wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" \
| sudo tee -a /etc/apt/sources.list
sudo apt-get update

sudo curl -sSL https://raw.githubus---ontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 Humble if needed:
```bash
sudo apt install -y ros-humble-desktop ros-dev-tools
```

## 3.Docker Installation
```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME:-$VERSION_CODENAME}) stable" | \
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

## 4. NVIDIA Driver Setup
```bash
cat /proc/driver/nvidia/version
sudo ubuntu-drivers list
sudo ubuntu-drivers install nvidia:570
```

Driver version 570 is stable and compatible with CUDA 12.8.

## 5. NVIDIA Container Toolkit Installation
```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends curl gnupg2

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update

export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.0-1
sudo apt-get install -y \
  nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
  nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
  libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
  libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl daemon-reload && sudo systemctl restart docker
```
## 6. Git LFS Installation
```bash
sudo apt-get install -y git-lfs
git lfs install --skip-repo
```
Git LFS is required for Isaac ROS repositories containing large model files such as .onnx, .engine, and .bin.


## 7. Isaac ROS Workspace Setup
```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
echo 'export ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev/"' >> ~/.bashrc
source ~/.bashrc
cd ~/workspaces/isaac_ros-dev/src

git clone --recursive -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone --recursive -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git
```

- isaac_ros_common: Core utilities for Docker build and environment setup

- isaac_ros_cumotion: GPU-based cuMotion motion planning

Ensure all repositories are checked out under release-3.2 for compatibility.

## 8.Doosan ROS 2 and cuMotion Integration
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/DoosanRobotics/doosan-robot2.git
git clone https://github.com/DoosanRobotics/doosanrobotics_cumotion_driver
```

----
# Runtime Execution Guide  
**Target:** Isaac ROS + Doosan ROS2 Integrated Environment  
**Scope:** Container Build/Execution → Launch Package → Command Publishing

---

## Step 1. Docker Build and Development Container Execution

### 1.1 Copy Docker Assets and Run Startup Script

`run_dev.sh` is the standard Isaac ROS script used to build and start a GPU-enabled development container.  
Before running it, ensure that the necessary Dockerfiles and scripts from the Doosan integration package are copied into the Isaac ROS common workspace.

```bash
# 1. Navigate to Doosan custom Docker assets
cd ~/ros2_ws/src/cumotion/dsr_cumotion/docker

# 2. Grant permission and execute startup script
chmod +x startup-doosan.sh
./startup-doosan.sh
```

**Description:**
`startup-doosan.sh` automatically copies the required Dockerfiles and helper scripts into
`~/workspaces/isaac_ros-dev/src/isaac_ros_common/`.
When the message `Copied ...` appears in the terminal, the copy process has completed successfully.

---

### 1.2 Build and Run the Development Container

Isaac ROS officially provides `isaac_ros_common/scripts/run_dev.sh` to build and launch its standard development container.
This handles GPU access, user mapping, and volume mounting automatically.

```bash
# 1. Move to the Isaac ROS common scripts directory
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts

# 2. Run the development container (includes initial image build)
./run_dev.sh
```

**Notes:**

* The provided `build-doosan.sh` and Dockerfiles assume that `ros2_ws` and `workspaces` are separate.
  If the build fails due to missing packages, refer to the internal Doosan documentation:
  [Doosan Robotics Wiki Reference](https://doosanrobotics.atlassian.net/wiki/spaces/1025409023/pages/3888939009/?draftShareId=1e53de4b-fa1e-47fa-86fc-401f6081d5d4)
* The first execution will take time while the Docker image is built.
* After launch, you will enter the container shell at `/workspaces/isaac_ros-dev` (default path).
* Verify GPU recognition inside the container:

```bash
nvidia-smi
```

A valid output confirms that the GPU is visible and configured correctly.

**Important:**

* Recommended driver: **570** (560 may fail during Docker builds; 580 may cause Isaac Sim compatibility issues).
* If the GPU is not detected inside the container, ensure the **nvidia-container-toolkit** is correctly configured (see environment setup).

---

### 1.3 ROS Environment Setup (Source Order)

To use ROS 2 and multiple workspaces correctly, source the environment in the following order:

```bash
# Base ROS 2 environment
source /opt/ros/humble/setup.bash

# Isaac ROS workspace
cd /workspaces/isaac_ros-dev
source install/setup.bash

# Doosan ROS2 workspace
cd /ros2_ws
source install/setup.bash
```
---

## Step 2. Launch Execution (Real / Virtual)

### 2.1 Main Launch File

The main entry point to start the integrated environment (Doosan + cuMotion)

**Real Robot Mode (connects to physical controller):**

```bash
ros2 launch dsr_cumotion start_cumotion.launch.py \
  mode:=real host:=192.168.137.100 enable_nvblox:=false gripper:=true
```

**Virtual Mode (connects to emulator or simulation):**

```bash
ros2 launch dsr_cumotion start_cumotion.launch.py \
  mode:=virtual host:=127.0.0.1 gripper:=true
```

**Parameter Description:**

* `mode` — `real` for physical robot, `virtual` for emulator/simulation.
* `host` — Controller IP (real robot) or emulator host (`127.0.0.1` for local).
* `gripper` — Must be `true` to load the VGC10-equipped model.
* `enable_cumotion` — Enables cuMotion-based motion planning (default: `true`).
* `enable_attach` — Enables object attachment handling (default: `true`).
* `use_sim_time` — `false` for real robot; `true` only for simulation environments.
* `obstacle` — `true` enables static obstacle generation via the PlanningScene node.

**Notes:**

* The current configuration supports the **M1013** model by default.

---

# Step 3. Command Publishing (Topic / Action)

## Overview
This section describes how to send motion commands to the Doosan robot through **dedicated topics** for each motion type.

All commands are internally processed by **MoveIt 2 + cuMotion + Doosan Controller**,  
and scaling factors (`max_vel_scale`, `max_acc_scale`) adjust the relative velocity and acceleration (range: `0.0–1.0`).

---

### 3-1. Pose Command (Euler)

**Topic:** `/target_pose`  
**Message type:** `dsr_cumotion_msgs/TargetPose`

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/msg/TargetPose "{
  x: 0.35, y: 0.20, z: 0.40,
  rx: 90.0, ry: 0.0, rz: 180.0,
  max_vel_scale: 0.5, max_acc_scale: 0.4
}" --once
```

* `rx`, `ry`, `rz`: Euler angles (degrees, ZYX order).  
* Defines an **absolute pose** in the robot’s base frame.  
* Use when specifying orientation in Euler form.

---

### 3-2. Joint Command

**Topic:** `/target_joint`  
**Message type:** `dsr_cumotion_msgs/TargetJoint`

```bash
ros2 topic pub /target_joint dsr_cumotion_msgs/msg/TargetJoint "{
  joint_position: [0.0, -90.0, 90.0, 0.0, 90.0, 0.0],
  max_vel_scale: 0.6,
  max_acc_scale: 0.4
}" --once
```

* Joint angles are specified in **degrees** (internally converted to radians).  
* Represents a **joint-space motion** request.

---

### 3-3. Named Command (Predefined Pose)

**Topic:** `/target_named`  
**Message type:** `dsr_cumotion_msgs/TargetNamed`

```bash
ros2 topic pub /target_named dsr_cumotion_msgs/msg/TargetNamed "{
  target_name: 'home',
  max_vel_scale: 0.8,
  max_acc_scale: 0.6
}" --once
```

* Executes a **predefined named pose** (e.g., `home`, `ready`, `grasp_pre`).  
* Named targets must be defined in the MoveIt SRDF configuration.

---

### 3-4. Relative Command (TCP and Base Frame)

**Topic:** `/target_relative`  
**Message type:** `dsr_cumotion_msgs/TargetRelative`

```bash
ros2 topic pub /target_relative dsr_cumotion_msgs/msg/TargetRelative "{
  reference_frame: 'tcp',
  dx: 0.0, dy: 0.00, dz: 0.20,
  drx: 0.0, dry: 0.0, drz: 0.0,
  max_vel_scale: 0.5,
  max_acc_scale: 0.5
}" --once
```

```bash
ros2 topic pub /target_relative dsr_cumotion_msgs/msg/TargetRelative "{
  reference_frame: 'base',
  dx: 0.10, dy: 0.00, dz: 0.00,
  drx: 0.0, dry: 0.0, drz: 0.0,
  max_vel_scale: 0.5,
  max_acc_scale: 0.5
}" --once
```

* Moves the robot **relative to the current TCP (tool frame)**.  
* `dx`, `dy`, `dz`: translational offsets in meters.  
* `drx`, `dry`, `drz`: rotational offsets in degrees.  
* Internally converted to an absolute pose in the `base_link` frame before execution.
---

### Summary

| Motion Type | Topic | Message Type | Description | Units |
|--------------|--------|---------------|--------------|--------|
| Pose | `/target_pose` | `TargetPose` | Absolute Cartesian pose command | m / deg |
| Joint | `/target_joint` | `TargetJoint` | Joint-space command | deg |
| Named | `/target_named` | `TargetNamed` | Move to predefined named pose | - |
| Relative | `/target_relative` | `TargetRelative` | Motion relative to current TCP | m / deg |

---

*Each command is handled sequentially by the `MoveCommandNode`,  
which dispatches it to the appropriate executor (`PoseExecutor`, `JointExecutor`, `NamedExecutor`, `RelativeExecutor`).*

---

## Optional: Object Attachment (Attach / Detach)

### Attach an Object to the Tool Frame

```bash
ros2 service call /attach_detach_command dsr_cumotion_msgs/srv/PickPlace "{motion_type: 0}"
```

---

### Detach Object

```bash
ros2 service call /attach_detach_command dsr_cumotion_msgs/srv/PickPlace "{motion_type: 1}"
```
---

**Author:** Runtime Execution Guide for Doosan ROS 2 + Isaac ROS
**Version:** Release 3.2 / ROS 2 Humble
