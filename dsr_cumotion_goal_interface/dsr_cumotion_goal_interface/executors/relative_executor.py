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
    """Executor for relative Cartesian movements (always in TCP/local coordinates)."""

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

        # TF listener for current end-effector pose lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def execute(self, msg, vel_scale=None, acc_scale=None, on_complete=None):
        """Compute target pose by applying Δ relative to the TCP (local tool) frame."""
        try:
            # Get current transform (base → tool)
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, Time()
            )

            current_pose = Pose()
            current_pose.position.x = tf_msg.transform.translation.x
            current_pose.position.y = tf_msg.transform.translation.y
            current_pose.position.z = tf_msg.transform.translation.z
            current_pose.orientation = tf_msg.transform.rotation

            # Extract translation & rotation deltas
            dx = getattr(msg, "dx", 0.0)
            dy = getattr(msg, "dy", 0.0)
            dz = getattr(msg, "dz", 0.0)
            drx = getattr(msg, "drx", 0.0)
            dry = getattr(msg, "dry", 0.0)
            drz = getattr(msg, "drz", 0.0)

            # Convert Δ position (TCP → world)
            q = current_pose.orientation
            R = _quaternion_to_matrix(q.x, q.y, q.z, q.w)
            delta_world = R.dot(np.array([dx, dy, dz]))

            # Compute absolute target pose
            target_pose = copy.deepcopy(current_pose)
            target_pose.position.x += delta_world[0]
            target_pose.position.y += delta_world[1]
            target_pose.position.z += delta_world[2]

            # Apply local (TCP) rotation offset if present
            if abs(drx) > 1e-6 or abs(dry) > 1e-6 or abs(drz) > 1e-6:
                qx, qy, qz, qw = euler_to_quaternion(
                    math.radians(drx), math.radians(dry), math.radians(drz)
                )
                R_delta = _quaternion_to_matrix(qx, qy, qz, qw)
                R_current = _quaternion_to_matrix(q.x, q.y, q.z, q.w)
                R_new = R_current.dot(R_delta)
                q_new = _matrix_to_quaternion(R_new)
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = q_new

            # Build temporary Pose message for PoseExecutor
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

            pose_msg.retry_num = getattr(msg, "retry_num", 0)

            # Logging
            self.node.get_logger().info(
                f"[RelativeExecutor] Δ(x,y,z)=({dx:.3f}, {dy:.3f}, {dz:.3f}) (TCP frame), retry_num={pose_msg.retry_num}"
            )

            # Execute using PoseExecutor
            return super().execute(
                pose_msg,
                vel_scale=vel_scale,
                acc_scale=acc_scale,
                on_complete=on_complete
            )

        except Exception as e:
            self.node.get_logger().error(f"[RelativeExecutor] Error: {e}")
            if on_complete:
                on_complete(False)
            return False
