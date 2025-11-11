
---

````markdown
# Step 3. Command Publishing (Topic / Action)

## Overview
This section describes how to send motion commands to the Doosan robot through the `/target_pose` topic.

- **Topic:** `/target_pose`  
- **Message type:** `dsr_cumotion/TargetPose2`  
- The command is processed internally by **MoveIt 2 + cuMotion + Doosan Controller**.  
- `max_vel_scale` and `max_acc_scale` define relative velocity and acceleration scaling (range: 0.0–1.0).

---

## 3-1. Pose Command (Euler)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'pose',
  x: 0.0, y: 0.0, z: 0.0,
  rx: 0.0, ry: 0.0, rz: 0.0,
  max_vel_scale: 0.5, max_acc_scale: 0.4}" --once
````

* `rx`, `ry`, `rz`: Euler angles (degrees, ZYX order).
* Use when defining orientation in Euler form.

---

## 3-2. Pose Command (Quaternion)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'pose',
  x: 0.0, y: 0.0, z: 0.0,
  qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0,
  max_vel_scale: 0.8, max_acc_scale: 0.6}" --once
```

* Define orientation using quaternion values.
* **Do not mix Euler and quaternion fields.**

---

## 3-3. Joint Command

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'joint',
  joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  max_vel_scale: 0.6, max_acc_scale: 0.4}" --once
```

* Joint angles are specified in **degrees**.
* Orientation fields are not required.

---

## 3-4. Named Command (Predefined Pose)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'named', name: 'HOME',
  max_vel_scale: 0.6, max_acc_scale: 0.5}" --once
```

* Executes a predefined pose (e.g., `HOME`, `SET`).

---

## 3-5. Relative Command (TCP Frame)

```bash
ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose \
"{move_type: 'relative',
  dx: 0.0, dy: 0.0, dz: 0.0,
  drx: 0.0, dry: 0.0, drz: 0.0,
  max_vel_scale: 0.5, max_acc_scale: 0.5}" --once
```

* Moves the robot **relative to the current TCP (tool frame)**.
* `dx`, `dy`, `dz`: translational offsets in meters.
* `drx`, `dry`, `drz`: rotational offsets in degrees.
* Internally converted to an absolute pose in the `base_link` frame before execution.

---

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
