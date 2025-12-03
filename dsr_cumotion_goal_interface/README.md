
---

# Step 3. Motion Command Publishing (Topic and Service Interfaces)

## Overview

This section describes how to send motion commands to the Doosan robot via ROS 2 topic and service interfaces.

### Topic Interface

* **Topic Name:** `/target_pose`
* **Message Type:** `dsr_cumotion_msgs/TargetPose`
* Motion commands published to this topic are processed through the following pipeline:
  **MoveIt 2 → cuMotion → Doosan Controller**
* `max_vel_scale` and `max_acc_scale` define relative velocity and acceleration scaling factors.

  * Valid range: `0.0` to `1.0`

---

## 3.1 Pose Command (Euler Representation)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'pose',
  x: 0.0, y: 0.0, z: 0.0,
  rx: 0.0, ry: 0.0, rz: 0.0,
  max_vel_scale: 0.5, max_acc_scale: 0.4}" --once
```

**Description**

* `rx`, `ry`, `rz` represent Euler angles in **degrees**.
* Rotation order follows the **ZYX convention**.
* Use this format when defining orientation using Euler angles.

---

## 3.2 Pose Command (Quaternion Representation)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'pose',
  x: 0.0, y: 0.0, z: 0.0,
  qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0,
  max_vel_scale: 0.8, max_acc_scale: 0.6}" --once
```

**Description**

* Orientation is defined using quaternion values.
* Euler and quaternion fields **must not be used together** in the same command.

---

## 3.3 Joint Space Command

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'joint',
  joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  max_vel_scale: 0.6, max_acc_scale: 0.4}" --once
```

**Description**

* Joint values must be specified in **degrees**.
* Position and orientation fields are not required for this command type.

---

## 3.4 Named Pose Command (Predefined Target)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'named', name: 'HOME',
  max_vel_scale: 0.6, max_acc_scale: 0.5}" --once
```

**Description**

* Executes a predefined named pose.
* Typical examples include `HOME`, `SET`, or other configured poses.

---

## 3.5 Relative Motion Command (TCP Frame)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'relative',
  dx: 0.0, dy: 0.0, dz: 0.0,
  drx: 0.0, dry: 0.0, drz: 0.0,
  max_vel_scale: 0.5, max_acc_scale: 0.5}" --once
```

**Description**

* The motion is executed **relative to the current TCP (tool frame)**.
* `dx`, `dy`, `dz`: translational offsets in **meters**
* `drx`, `dry`, `drz`: rotational offsets in **degrees**
* The relative command is internally converted into an absolute pose in the `base_link` frame before execution.

---

# Step 4. Pick and Place Service Interface

The following service interface triggers a predefined pick-and-place sequence.

* **Service Name:** `/pick_place_command`
* **Service Type:** `dsr_cumotion_msgs/srv/PickPlace`

---

## 4.1 Pick Operation Command

```bash
ros2 service call /pick_place_command dsr_cumotion_msgs/srv/PickPlace "{
  mode: 0,
  dx: 0.0,
  dy: 0.0,
  dz: -0.10,
  drx: 0.0,
  dry: 0.0,
  drz: 0.0,
  vel: 0.5,
  acc: 0.5,
  sequence: 1
}"
```

**Description**

* `mode: 0` selects **pick operation mode**.
* Relative motion offsets are applied before execution.
* `vel` and `acc` define velocity and acceleration scaling.
* `sequence` specifies the execution step index.

---

## 4.2 Place Operation Command

```bash
ros2 service call /pick_place_command dsr_cumotion_msgs/srv/PickPlace "{
  mode: 1,
  dx: 0.0,
  dy: 0.0,
  dz: -0.10,
  drx: 0.0,
  dry: 0.0,
  drz: 0.0,
  vel: 0.5,
  acc: 0.5,
  sequence: 1
}"
```

**Description**

* `mode: 1` selects **place operation mode**.
* Other parameters follow the same convention as the pick command.

---