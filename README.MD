
---
# Doosan ROS2 + Isaac ROS Integrated Environment Setup Guide

This guide describes how to configure and build a unified development environment that integrates **Doosan ROS2** with **NVIDIA Isaac ROS** using Docker.

---

## 1. Placing the `docker` / `scripts` Directories

First, copy the following directories required for Doosan integration  
from `ros2_ws/src/cumotion/` to the **parent directory of the Isaac ROS workspace (`ISAAC_ROS_WS/../`)**.

### Directories to Copy

- `docker/`
- `scripts/`

### Example Copy Commands

```bash
cp -r ~/ros2_ws/src/cumotion/docker   ~/workspaces/
cp -r ~/ros2_ws/src/cumotion/scripts ~/workspaces/
````

### Example Directory Structure After Copy

```text
workspaces/
├── isaac_ros-dev/          # ISAAC_ROS_WS
├── docker/
└── scripts/
```

---

## 2. Creating the Isaac ROS CLI Configuration File

Create the Isaac ROS CLI configuration directory using the following command:

```bash
mkdir -p ~/workspaces/isaac_ros-dev/.isaac-ros-cli
```

Then create a `config.yaml` file with the following content:

### `~/workspaces/isaac_ros-dev/.isaac-ros-cli/config.yaml`

```yaml
environment:
  mode: docker

docker:
  image:
    additional_image_keys:
      - doosan
```

### Configuration Description

This configuration ensures that Docker image layers are combined in the following order during the build process:

1. **noble** – Base OS + CUDA
2. **ros2_jazzy** – Isaac ROS ROS2 Layer
3. **doosan** – User-defined Doosan Integration Layer

---

## 3. Building the Docker Image

Once the setup is complete, run the following command to generate the integrated Isaac + Doosan development image:

```bash
isaac-ros activate --build-local
```

### What Happens After Execution

* Dockerfile layers are merged in the following order:
  `noble → ros2_jazzy → doosan`
* A new integrated Docker image is automatically created
* The container is automatically launched
* Scripts inside `/usr/local/bin/scripts/entrypoint_additions` are executed
* The `00-build-doosan.user.sh` script is automatically executed, which triggers:
  **A full `colcon build` of the entire Isaac ROS + Doosan ROS2 workspace**

---

