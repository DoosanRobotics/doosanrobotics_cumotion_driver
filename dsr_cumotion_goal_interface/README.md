# Step 3. Command Publishing (Topic / Action)

## Overview
This section describes how to send motion commands to the Doosan robot through **dedicated topics** for each motion type.

All commands are internally processed by **MoveIt 2 + cuMotion + Doosan Controller**,  
and scaling factors (`max_vel_scale`, `max_acc_scale`) adjust the relative velocity and acceleration (range: `0.0–1.0`).

---

## 3-1. Pose Command (Euler)

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

## 3-2. Joint Command

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

## 3-3. Named Command (Predefined Pose)

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

## 3-4. Relative Command

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

## Summary

| Motion Type | Topic | Message Type | Description | Units |
|--------------|--------|---------------|--------------|--------|
| Pose | `/target_pose` | `TargetPose` | Absolute Cartesian pose command | m / deg |
| Joint | `/target_joint` | `TargetJoint` | Joint-space command | deg |
| Named | `/target_named` | `TargetNamed` | Move to predefined named pose | - |
| Relative | `/target_relative` | `TargetRelative` | Motion relative to current TCP | m / deg |

---

*Each command is handled sequentially by the `MoveCommandNode`,  
which dispatches it to the appropriate executor (`PoseExecutor`, `JointExecutor`, `NamedExecutor`, `RelativeExecutor`).*
