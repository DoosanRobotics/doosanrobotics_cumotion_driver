# Isaac ROS + Doosan ROS 2 + cuMotion Integrated Environment Setup Guide

This document describes the complete procedure to configure and build an integrated development environment combining **NVIDIA Isaac ROS (Release 4.0)**, **Jetson AGX Thor**, and **Doosan ROS 2 (Jazzy)** for GPU-accelerated motion planning using **cuMotion** and **MoveIt 2**.

---

# Part 1. Environment Setup

This section describes the preparation of the Jetson platform, Isaac ROS workspace, and Doosan ROS 2 integration environment.

## 1.1 System Environment

| Component | Version         |
| --------- | --------------- |
| Isaac ROS | Release 4.0     |
| JetPack   | 7.0             |
| ROS 2     | Jazzy           |
| Hardware  | Jetson AGX Thor |


## 1.2 Jetson Thor and JetPack Preparation

The following steps must be completed **in order** before proceeding.

### 1.3 Jetson Thor Flashing and JetPack Installation

Follow the official NVIDIA documentation to flash Jetson Thor and install JetPack 7.0:

[https://docs.nvidia.com/jetson/agx-thor-devkit/user-guide/latest/index.html](https://docs.nvidia.com/jetson/agx-thor-devkit/user-guide/latest/index.html)

### 1.3 Isaac ROS Setup for Jetson Thor

Complete the Isaac ROS environment setup for Jetson Thor using the following documentation:

[https://nvidia-isaac-ros.github.io/getting_started/index.html](https://nvidia-isaac-ros.github.io/getting_started/index.html)

#### Important Note

For this project, the Isaac ROS workspace must be configured as follows:

```bash
export ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev/"
```


## 1.4 Isaac ROS Workspace Creation

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
echo 'export ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev/"' >> ~/.bashrc
source ~/.bashrc
cd ~/workspaces/isaac_ros-dev/src
```


## 1.5 Isaac ROS and cuMotion Source Installation (Release 4.0)

```bash
cd ~/workspaces/isaac_ros-dev/src

git clone --recurse-submodules -b release-4.0 \
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common

git clone --recurse-submodules -b release-4.0 \
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git isaac_ros_cumotion
```

| Repository           | Description                                           |
| -------------------- | ----------------------------------------------------- |
| `isaac_ros_common`   | Core utilities for Docker build and environment setup |
| `isaac_ros_cumotion` | GPU-based cuMotion motion planning pipeline           |

All repositories **must be checked out to the `release-4.0` branch** to ensure compatibility.



## 1.6 Doosan ROS 2 and cuMotion Driver Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone -b jazzy https://github.com/DoosanRobotics/doosan-robot2.git
git clone -b jetson_thor https://github.com/DoosanRobotics/doosanrobotics_cumotion_driver.git
```



## 1.7 Docker-Based Isaac ROS and Doosan ROS 2 Integration

### 7.1 Copying the `docker/` and `scripts/` Directories

```bash
cp -r ~/ros2_ws/src/doosanrobotics_cumotion_driver/docker   ~/workspaces/
cp -r ~/ros2_ws/src/doosanrobotics_cumotion_driver/scripts ~/workspaces/
```

### 7.2 Expected Directory Structure

```text
workspaces/
├── isaac_ros-dev/
├── docker/
└── scripts/
```

---

## 8. Isaac ROS CLI Configuration

```bash
mkdir -p ~/workspaces/isaac_ros-dev/.isaac-ros-cli
```

### `config.yaml`

```yaml
environment:
  mode: docker

docker:
  image:
    additional_image_keys:
      - doosan
```

---

## 9. Docker Image Layer Composition

1. `noble` — Base Ubuntu OS with CUDA
2. `ros2_jazzy` — Isaac ROS Jazzy layer
3. `doosan` — Doosan ROS 2 integration layer

---

## 10. Building the Integrated Docker Image

```bash
isaac-ros activate --build-local
```

---

## 11. Post-Build Automatic Procedures

* Integrated Docker image build
* Automatic container launch
* Execution of `/usr/local/bin/scripts/entrypoint_additions`
* Execution of `00-build-doosan.user.sh`
* Full `colcon build` of the entire workspace

---

# Part 2. Launch Execution and Robot Command Interfaces

This section describes how to launch the integrated system and send motion commands.

---

## 12. Launch Execution (Real / Virtual Modes)

### 12.1 Real Robot Mode

```bash
ros2 launch dsr_cumotion start_cumotion.launch.py \
  mode:=real host:=192.168.137.100 gripper:=true
```

### 12.2 Virtual Mode (Emulator / Simulation)

```bash
ros2 launch dsr_cumotion start_cumotion.launch.py \
  mode:=virtual host:=127.0.0.1 gripper:=true
```

---

## 12.3 Launch Parameter Description

| Parameter         | Description                                       |
| ----------------- | ------------------------------------------------- |
| `mode`            | `real` for physical robot, `virtual` for emulator |
| `host`            | Controller IP or emulator host                    |
| `gripper`         | Enables VGC10 gripper model                       |
| `enable_cumotion` | cuMotion planner enable flag                      |
| `enable_attach`   | Object attachment enable flag                     |
| `use_sim_time`    | Simulation time usage                             |
| `obstacle`        | PlanningScene obstacle generation                 |

---

## 13. Motion Command Publishing (Topic Interface)

* **Topic:** `/target_pose`
* **Type:** `dsr_cumotion_msgs/TargetPose`
* Pipeline: **MoveIt 2 → cuMotion → Doosan Controller**

### 13.1 Pose (Euler)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'pose', x: 0.0, y: 0.0, z: 0.0, rx: 0.0, ry: 0.0, rz: 0.0, max_vel_scale: 0.5, max_acc_scale: 0.4}" --once
```

### 13.2 Pose (Quaternion)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'pose', x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0, max_vel_scale: 0.8, max_acc_scale: 0.6}" --once
```

### 13.3 Joint

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'joint', joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], max_vel_scale: 0.6, max_acc_scale: 0.4}" --once
```

### 13.4 Named

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'named', name: 'HOME', max_vel_scale: 0.6, max_acc_scale: 0.5}" --once
```

### 13.5 Relative

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'relative', dx: 0.0, dy: 0.0, dz: 0.0, drx: 0.0, dry: 0.0, drz: 0.0, max_vel_scale: 0.5, max_acc_scale: 0.5}" --once
```

---

## 14. Pick and Place Service Interface

* **Service:** `/pick_place_command`
* **Type:** `dsr_cumotion_msgs/srv/PickPlace`

### 14.1 Pick

```bash
ros2 service call /pick_place_command dsr_cumotion_msgs/srv/PickPlace "{mode: 0, dx: 0.0, dy: 0.0, dz: -0.10, drx: 0.0, dry: 0.0, drz: 0.0, vel: 0.5, acc: 0.5, sequence: 1}"
```

### 14.2 Place

```bash
ros2 service call /pick_place_command dsr_cumotion_msgs/srv/PickPlace "{mode: 1, dx: 0.0, dy: 0.0, dz: -0.10, drx: 0.0, dry: 0.0, drz: 0.0, vel: 0.5, acc: 0.5, sequence: 1}"
```

---

## 15. Simple Attach / Detach Service

```bash
# Attach
ros2 service call /attach_detach_command dsr_cumotion_msgs/srv/PickPlace "{motion_type: 0}"

# Detach
ros2 service call /attach_detach_command dsr_cumotion_msgs/srv/PickPlace "{motion_type: 1}"
```

---

# Part 3. Package Description and System Operational Notes

This section describes the **core software stack**, the **high-level functional roles of the major packages**, and the **operational notes and system limitations** of the
**Doosan + Isaac ROS + cuMotion integrated system**.

---

## 3.1 Core Software Stack

| Component    | Description                                        |
| ------------ | -------------------------------------------------- |
| Doosan ROS 2 | Doosan robot hardware interface                    |
| MoveIt 2     | Motion planning and trajectory execution framework |
| cuMotion     | NVIDIA GPU-accelerated motion planner              |
| Isaac ROS    | CUDA-optimized robotics middleware                 |

---

## 3.2 `dsr_cumotion`

**Role: Core Integration Package for Doosan, MoveIt 2, and cuMotion**

The `dsr_cumotion` package is the **central integration layer** of the system.
It connects the following components into a single execution pipeline:

* Doosan ROS 2 hardware interface
* MoveIt 2 motion planning framework
* NVIDIA cuMotion planner execution
* Pick-and-place task server
* Planning scene and static obstacle management

### Key Responsibilities

* Provides the **main system launch entry point** (`start_cumotion.launch.py`)
* Configures the **cuMotion and MoveIt 2 planning pipelines**
* Manages **robot model integration**

  * URDF
  * SRDF
  * XRDF
* Provides a **Pick-and-Place task execution server**
* Manages **static obstacles using the Planning Scene**
* Manages workspace boundaries (workbound)

This package directly controls the **core motion planning and execution behavior of the robot**.

---

## 3.3 `dsr_cumotion_goal_interface`

**Role: Motion Command Dispatch Interface**

The `dsr_cumotion_goal_interface` package receives high-level user commands and
acts as the **command gateway that forwards them to the MoveIt 2 + cuMotion execution pipeline**.

### Key Responsibilities

* Subscribes to the `/target_pose` topic
* Selects the appropriate execution strategy based on the command type:

  * Absolute pose motion
  * Joint-space motion
  * Named pose motion
  * Relative TCP motion
* Sends motion goals to the **MoveIt 2 Action Server**
* Monitors execution status and feedback
* Executes commands sequentially using a **multi-command queue**

This package serves as the **intermediate control layer between user commands and physical robot execution**.

---

## 3.4 `dsr_cumotion_msgs`

**Role: System-Wide Message and Service Interface Definition Package**

The `dsr_cumotion_msgs` package defines all **custom ROS 2 messages and service types** used throughout the system for motion and task-level control.

### Key Responsibilities

* Defines the **unified motion command message** that supports:

  * Absolute pose commands
  * Joint commands
  * Named target commands
  * Relative TCP commands
* Defines the **Pick-and-Place task control service interface**, including:

  * Approach → attach → retreat sequence
  * Approach → detach → retreat sequence
* Provides the **standard API contract** between:

  * User applications
  * Command interface nodes
  * Planning and execution subsystems

---

## 3.5 System Notes and Limitations

* The default supported robot model is **M1013**.
* To use other robot models, additional configuration is required:

  * URDF
  * SRDF
  * cuMotion XRDF
* For real robot operation, the following conditions **must be satisfied**:

  * The emergency stop (E-Stop) must be released
  * The network connection to the robot controller must be stable
  * The robot servo power must be enabled

---