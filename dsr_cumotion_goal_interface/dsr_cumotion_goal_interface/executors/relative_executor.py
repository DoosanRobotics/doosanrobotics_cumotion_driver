import copy
import math
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from ..utils.math_utils import (
    euler_to_quaternion,
    _quaternion_to_matrix,
    _matrix_to_quaternion,
)
from .pose_executor import PoseExecutor


class RelativeExecutor(PoseExecutor):
    """Executor for relative Cartesian movements in TCP (local) coordinates."""

    def __init__(
        self,
        node,
        group_name,
        pipeline_id,
        base_frame,
        tool_frame,
        planner_id="cuMotion",
        allowed_planning_time=5.0,
        num_planning_attempts=10,
        default_vel_scale=1.0,
        default_acc_scale=1.0,
    ):
        super().__init__(
            node,
            group_name,
            pipeline_id,
            base_frame,
            tool_frame,
            planner_id=planner_id,
            allowed_planning_time=allowed_planning_time,
            num_planning_attempts=num_planning_attempts,
            default_vel_scale=default_vel_scale,
            default_acc_scale=default_acc_scale,
        )

        # Initialize TF listener for current end-effector pose lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def execute(self, msg, vel_scale=None, acc_scale=None):
        """Compute target pose by applying a relative Δ to the current TCP pose (in local tool frame)."""
        try:

            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.base_frame, self.tool_frame, Time()
                )
            except TransformException as ex:
                self.node.get_logger().error(f"[RelativeExecutor] TF lookup failed: {ex}")
                return False

            current_pose = Pose()
            current_pose.position.x = tf_msg.transform.translation.x
            current_pose.position.y = tf_msg.transform.translation.y
            current_pose.position.z = tf_msg.transform.translation.z
            current_pose.orientation = tf_msg.transform.rotation

            dx = getattr(msg, "dx", getattr(msg, "x", 0.0))
            dy = getattr(msg, "dy", getattr(msg, "y", 0.0))
            dz = getattr(msg, "dz", getattr(msg, "z", 0.0))
            drx = getattr(msg, "drx", getattr(msg, "rx", 0.0))
            dry = getattr(msg, "dry", getattr(msg, "ry", 0.0))
            drz = getattr(msg, "drz", getattr(msg, "rz", 0.0))

            q = current_pose.orientation
            R = _quaternion_to_matrix(q.x, q.y, q.z, q.w)

            delta_local = np.array([dx, dy, dz])
            delta_world = R.dot(delta_local)

            # Compute final absolute pose (base frame)
            target_pose = copy.deepcopy(current_pose)
            target_pose.position.x += delta_world[0]
            target_pose.position.y += delta_world[1]
            target_pose.position.z += delta_world[2]

            # Apply local rotation offset (if any)
            if abs(drx) > 1e-6 or abs(dry) > 1e-6 or abs(drz) > 1e-6:
                qx, qy, qz, qw = euler_to_quaternion(
                    math.radians(drx), math.radians(dry), math.radians(drz)
                )
                R_delta = _quaternion_to_matrix(qx, qy, qz, qw)
                R_new = R.dot(R_delta)
                q_new = _matrix_to_quaternion(R_new)
                target_pose.orientation.x = q_new[0]
                target_pose.orientation.y = q_new[1]
                target_pose.orientation.z = q_new[2]
                target_pose.orientation.w = q_new[3]

            # Prepare temporary Pose message for PoseExecutor
            pose_msg = type("Tmp", (), {})()
            pose_msg.x = target_pose.position.x
            pose_msg.y = target_pose.position.y
            pose_msg.z = target_pose.position.z
            pose_msg.qx = target_pose.orientation.x
            pose_msg.qy = target_pose.orientation.y
            pose_msg.qz = target_pose.orientation.z
            pose_msg.qw = target_pose.orientation.w
            pose_msg.max_vel_scale = getattr(msg, "max_vel_scale", 1.0)
            pose_msg.max_acc_scale = getattr(msg, "max_acc_scale", 1.0)

            # Logging and delegate to PoseExecutor
            self.node.get_logger().info(
                f"[RelativeExecutor] Local Δ(x,y,z)=({dx:.3f}, {dy:.3f}, {dz:.3f}) → "
                f"Target=({pose_msg.x:.3f}, {pose_msg.y:.3f}, {pose_msg.z:.3f})"
            )

            # Execute as an absolute pose move using PoseExecutor
            return super().execute(pose_msg, vel_scale, acc_scale)

        except Exception as e:
            self.node.get_logger().error(f"[RelativeExecutor] Error: {e}")
            return False
