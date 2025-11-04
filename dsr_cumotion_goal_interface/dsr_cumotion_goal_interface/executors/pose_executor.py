import math
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)
from shape_msgs.msg import SolidPrimitive
from .base_executor import MoveItExecutorBase
from ..utils.math_utils import euler_to_quaternion


class PoseExecutor(MoveItExecutorBase):
    """Executor for Cartesian pose-based motion commands"""

    def __init__(
        self, node, group_name, pipeline_id, base_frame, tool_frame,
        planner_id="cuMotion", allowed_planning_time=5.0, num_planning_attempts=10,
        default_vel_scale=1.0, default_acc_scale=1.0
    ):
        super().__init__(node, group_name, pipeline_id, base_frame, tool_frame)
        self.planner_id = planner_id
        self.allowed_planning_time = allowed_planning_time
        self.num_planning_attempts = num_planning_attempts
        self.default_vel_scale = default_vel_scale
        self.default_acc_scale = default_acc_scale
        self.current_goal_handle = None

    def execute(self, msg, vel_scale=None, acc_scale=None):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = msg.x, msg.y, msg.z

        if hasattr(msg, "rx") and hasattr(msg, "ry") and hasattr(msg, "rz") and (
            msg.rx or msg.ry or msg.rz
        ):
            qx, qy, qz, qw = euler_to_quaternion(
                math.radians(msg.rx),
                math.radians(msg.ry),
                math.radians(msg.rz),
            )
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
                qx, qy, qz, qw
            )
        else:
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
                msg.qx, msg.qy, msg.qz, msg.qw
            )

        vel_scale = vel_scale if vel_scale is not None else getattr(msg, "max_vel_scale", self.default_vel_scale)
        acc_scale = acc_scale if acc_scale is not None else getattr(msg, "max_acc_scale", self.default_acc_scale)
        if vel_scale <= 0.0:
            vel_scale = self.default_vel_scale
        if acc_scale <= 0.0:
            acc_scale = self.default_acc_scale

        self.node.get_logger().info(
            f"[PoseExecutor] Executing pose move with vel_scale={vel_scale:.2f}, acc_scale={acc_scale:.2f}"
        )

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = self.planner_id
        req.allowed_planning_time = float(self.allowed_planning_time)
        req.num_planning_attempts = int(self.num_planning_attempts)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)

        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base_frame
        pos_c.link_name = self.tool_frame
        pos_c.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.001, 0.001, 0.001])
        ]
        pos_c.constraint_region.primitive_poses = [pose]
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.base_frame
        ori_c.link_name = self.tool_frame
        ori_c.orientation = pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.1
        ori_c.absolute_y_axis_tolerance = 0.1
        ori_c.absolute_z_axis_tolerance = 0.1
        ori_c.weight = 1.0

        goal = Constraints(
            position_constraints=[pos_c],
            orientation_constraints=[ori_c],
        )
        req.goal_constraints = [goal]

        return self._send_goal(req, "Pose move")
