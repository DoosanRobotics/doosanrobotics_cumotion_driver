#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveToPoseNode (Unified Version)
--------------------------------
- Supports both MoveIt pose goals and direct joint trajectory goals.
- Integrates with moveit_simple_controller_manager (dsr_moveit_controller)
- Handles orientation via:
    • orientation_id (predefined quaternion)
    • quaternion (qx, qy, qz, qw)
    • Euler angles (rx, ry, rz; deg or rad)
"""

import math
import threading
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

# --- MoveIt ---
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# --- Direct Trajectory ---
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# --- Custom Msg ---
from dsr_cumotion.msg import TargetPose

# --- TF ---
from tf_transformations import quaternion_from_euler

# =========================================================================== #
#                               CONSTANTS
# =========================================================================== #

MOVE_RESULT_WAIT_SEC = 120.0
DEFAULT_SERVER_WAIT_SEC = 5.0
POS_TOL = 0.05
ORI_TOL = 0.1

# Orientation lookup table
ORIENTATION_TABLE: Dict[int, Tuple[float, float, float, float]] = {
    1: (0.7071, 0.7071, 0.0, 0.0),
    2: (1.0, 0.0, 0.0, 0.0),
    3: (0.0, 1.0, 0.0, 0.0),
    4: (0.7071, -0.7071, 0.0, 0.0),
    5: (0.0, 0.7071, 0.7071, 0.0),
    6: (-0.7071, 0.0, 0.0, 0.7071),
    7: (0.7071, 0.0, 0.0, 0.7071),
    8: (0.0, 0.7071, -0.7071, 0.0),
    9: (1.0, 1.0, 1.0, 1.0),
    10: (-1.0, 1.0, -1.0, 1.0),
}

# =========================================================================== #
#                               UTILITIES
# =========================================================================== #

def normalize_quat(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/n, y/n, z/n, w/n) if n > 0 else (0.0, 0.0, 0.0, 1.0)

def deg2rad(deg): return deg * math.pi / 180.0
def rad2deg(rad): return rad * 180.0 / math.pi
def movegroup_success(result: MoveGroup.Result) -> bool:
    return result.error_code.val == MoveItErrorCodes.SUCCESS


# =========================================================================== #
#                               MAIN NODE
# =========================================================================== #

class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__("move_to_pose_node")
        self.cb_group = ReentrantCallbackGroup()
        self._move_lock = threading.Lock()

        # Parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("group_name", "manipulator"),
                ("pipeline_id", "isaac_ros_cumotion"),
                ("planner_id", "cuMotion"),
                ("base_frame", "base_link"),
                ("eef_link", "grasp_frame"),
                ("tool_frame", "grasp_frame"),
                ("allowed_planning_time", 10.0),
                ("num_planning_attempts", 50),
                ("max_vel_scale", 1.0),
                ("max_acc_scale", 1.0),
                ("joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]),
                ("trajectory_controller_name", "/dsr_joint_trajectory_controller/follow_joint_trajectory"),
            ],
        )

        # Action Clients
        self._move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self._traj_client = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter("trajectory_controller_name").value,
            callback_group=self.cb_group
        )

        # Topic subscription
        self.create_subscription(TargetPose, "/target_pose", self._on_target_pose, 10)
        self.get_logger().info("[MoveToPoseNode] ✅ Ready. Publish TargetPose to /target_pose.")

    # ======================================================================= #
    #                              TOPIC CALLBACK
    # ======================================================================= #

    def _on_target_pose(self, msg: TargetPose):
        if not self._move_lock.acquire(blocking=False):
            self.get_logger().warn("Busy - ignoring new command.")
            return
        threading.Thread(target=self._run_move_thread, args=(msg,), daemon=True).start()

    def _run_move_thread(self, msg: TargetPose):
        try:
            # 1️⃣ Direct joint command
            if len(msg.joint_values) > 0:
                success = self._move_to_joints(msg.joint_values)
                self.get_logger().info("[JointMove] ✅ Success" if success else "[JointMove] ❌ Failed")
                return

            # 2️⃣ Orientation handling
            q = self._resolve_orientation(msg)
            if q is None:
                self.get_logger().error("❌ No valid orientation info found.")
                return

            # 3️⃣ Build pose
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = msg.x, msg.y, msg.z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

            self.get_logger().info(
                f"[MoveToPose] Target pose: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}, quat={tuple(round(v,3) for v in q)}"
            )

            success = self._move_to_pose(pose)
            self.get_logger().info("[MoveGroup] ✅ Success" if success else "[MoveGroup] ❌ Failed")

        finally:
            self._move_lock.release()

    # ======================================================================= #
    #                              ORIENTATION
    # ======================================================================= #

    def _resolve_orientation(self, msg: TargetPose):
        # ① Orientation ID
        if msg.orientation_id in ORIENTATION_TABLE:
            return normalize_quat(*ORIENTATION_TABLE[msg.orientation_id])

        # ② Quaternion directly
        if any(abs(v) > 1e-6 for v in [msg.qx, msg.qy, msg.qz, msg.qw]):
            return normalize_quat(msg.qx, msg.qy, msg.qz, msg.qw)

        # ③ Euler angles
        if any(abs(v) > 1e-6 for v in [msg.rx, msg.ry, msg.rz]):
            rx, ry, rz = msg.rx, msg.ry, msg.rz
            if abs(rx) > 2*math.pi or abs(ry) > 2*math.pi or abs(rz) > 2*math.pi:
                rx, ry, rz = map(deg2rad, [rx, ry, rz])
            return quaternion_from_euler(rx, ry, rz)

        return None

    # ======================================================================= #
    #                              JOINT TRAJECTORY
    # ======================================================================= #

    def _move_to_joints(self, joint_values):
        if not self._traj_client.wait_for_server(timeout_sec=DEFAULT_SERVER_WAIT_SEC):
            self.get_logger().error("❌ Trajectory controller not available.")
            return False

        joint_names = self.get_parameter("joint_names").value
        if len(joint_values) != len(joint_names):
            self.get_logger().error("❌ Joint count mismatch.")
            return False

        # deg → rad 변환 자동 처리
        if any(abs(v) > 2 * math.pi for v in joint_values):
            joint_values = [deg2rad(v) for v in joint_values]
            self.get_logger().info("Interpreted as degrees → converted to radians.")

        # Trajectory message 생성
        traj = JointTrajectory()
        traj.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = joint_values
        pt.velocities = [0.0] * len(joint_values)
        pt.accelerations = [0.0] * len(joint_values)
        pt.time_from_start.sec = 3
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        done = threading.Event()
        result_ok = {"v": False}

        def on_goal_response(fut):
            try:
                gh = fut.result()
                if not gh.accepted:
                    self.get_logger().error("❌ Joint goal rejected.")
                    done.set()
                    return

                def on_result(res_future):
                    try:
                        res = res_future.result().result
                        result_ok["v"] = (res.error_code == 0)
                    except Exception as e:
                        self.get_logger().error(f"get_result failed: {e}")
                    finally:
                        done.set()
                gh.get_result_async().add_done_callback(on_result)
            except Exception as e:
                self.get_logger().error(f"send_goal failed: {e}")
                done.set()

        self._traj_client.send_goal_async(goal).add_done_callback(on_goal_response)
        done.wait(timeout=20.0)
        return result_ok["v"]

    # ======================================================================= #
    #                              MOVEIT POSE GOAL
    # ======================================================================= #

    def _move_to_pose(self, target_pose: Pose):
        if not self._move_client.wait_for_server(timeout_sec=DEFAULT_SERVER_WAIT_SEC):
            self.get_logger().error("❌ MoveGroup server not available.")
            return False

        goal = self._build_goal(target_pose)
        done = threading.Event()
        result_ok = {"v": False}

        def on_goal_response(fut):
            try:
                gh = fut.result()
                if not gh.accepted:
                    self.get_logger().error("❌ Pose goal rejected.")
                    done.set()
                    return

                def on_result(res_future):
                    try:
                        res = res_future.result().result
                        result_ok["v"] = movegroup_success(res)
                    except Exception as e:
                        self.get_logger().error(f"get_result failed: {e}")
                    finally:
                        done.set()
                gh.get_result_async().add_done_callback(on_result)
            except Exception as e:
                self.get_logger().error(f"send_goal failed: {e}")
                done.set()

        self._move_client.send_goal_async(goal).add_done_callback(on_goal_response)
        done.wait(timeout=MOVE_RESULT_WAIT_SEC)
        return result_ok["v"]

    # ======================================================================= #
    #                              BUILD GOAL
    # ======================================================================= #

    def _build_goal(self, target_pose: Pose):
        p = self.get_parameter
        goal = MoveGroup.Goal()
        goal.request.group_name = p("group_name").value
        goal.request.pipeline_id = p("pipeline_id").value
        goal.request.planner_id = p("planner_id").value
        goal.request.num_planning_attempts = int(p("num_planning_attempts").value)
        goal.request.allowed_planning_time = float(p("allowed_planning_time").value)
        goal.request.max_velocity_scaling_factor = float(p("max_vel_scale").value)
        goal.request.max_acceleration_scaling_factor = float(p("max_acc_scale").value)

        pos_c = PositionConstraint()
        pos_c.header.frame_id = p("base_frame").value
        pos_c.link_name = p("eef_link").value
        pos_c.weight = 1.0
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[POS_TOL]*3)
        bv = BoundingVolume()
        bv.primitives.append(prim)
        bv.primitive_poses.append(target_pose)
        pos_c.constraint_region = bv

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = p("base_frame").value
        ori_c.link_name = p("tool_frame").value
        ori_c.orientation = target_pose.orientation
        ori_c.absolute_x_axis_tolerance = ORI_TOL
        ori_c.absolute_y_axis_tolerance = ORI_TOL
        ori_c.absolute_z_axis_tolerance = ORI_TOL
        ori_c.weight = 1.0

        cs = Constraints()
        cs.position_constraints.append(pos_c)
        cs.orientation_constraints.append(ori_c)
        goal.request.goal_constraints = [cs]
        return goal


# =========================================================================== #
#                               MAIN
# =========================================================================== #

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
