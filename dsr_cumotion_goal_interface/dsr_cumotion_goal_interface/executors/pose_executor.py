import math
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from .base_executor import MoveItExecutorBase
from ..utils.math_utils import euler_to_quaternion


class PoseExecutor(MoveItExecutorBase):
    """Executor for Cartesian pose-based motion commands (simplified & modernized)."""

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
        super().__init__(node, group_name, pipeline_id, base_frame, tool_frame)

        self.planner_id = planner_id
        self.allowed_planning_time = allowed_planning_time
        self.num_planning_attempts = num_planning_attempts
        self.default_vel_scale = default_vel_scale
        self.default_acc_scale = default_acc_scale

        # Cache the latest joint states to define explicit start_state
        self.latest_joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg):
        """Callback to store the most recent joint state for planning start state."""
        self.latest_joint_state = msg

    def execute(self, msg, vel_scale=None, acc_scale=None):
        """Build MotionPlanRequest from pose message and send to MoveIt2."""

        # Pose construction
        pose = Pose()
        pose.position.x = msg.x
        pose.position.y = msg.y
        pose.position.z = msg.z

        # Orientation: prefer Euler if provided, otherwise quaternion
        if hasattr(msg, "rx") and hasattr(msg, "ry") and hasattr(msg, "rz") and (msg.rx or msg.ry or msg.rz):
            qx, qy, qz, qw = euler_to_quaternion(
                math.radians(msg.rx),
                math.radians(msg.ry),
                math.radians(msg.rz),
            )
        else:
            qx, qy, qz, qw = msg.qx, msg.qy, msg.qz, msg.qw

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # Velocity / acceleration scaling
        vel_scale = (
            vel_scale
            if vel_scale is not None
            else getattr(msg, "max_vel_scale", self.default_vel_scale)
        )
        acc_scale = (
            acc_scale
            if acc_scale is not None
            else getattr(msg, "max_acc_scale", self.default_acc_scale)
        )
        if vel_scale <= 0.0:
            vel_scale = self.default_vel_scale
        if acc_scale <= 0.0:
            acc_scale = self.default_acc_scale

        self.node.get_logger().info(
            f"[PoseExecutor] Executing pose move "
            f"(vel_scale={vel_scale:.2f}, acc_scale={acc_scale:.2f})"
        )

        # Build MotionPlanRequest
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = self.planner_id
        req.allowed_planning_time = float(self.allowed_planning_time)
        req.num_planning_attempts = int(self.num_planning_attempts)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)

        # Add explicit start_state if available
        if self.latest_joint_state:
            start_state = RobotState()
            start_state.joint_state = self.latest_joint_state
            req.start_state = start_state
            self.node.get_logger().info(
                f"[PoseExecutor] Using explicit start_state with {len(self.latest_joint_state.name)} joints."
            )
        else:
            self.node.get_logger().warn(
                "[PoseExecutor] No joint_state received yet — using MoveIt default start state."
            )

        # Position + Orientation constraints
        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base_frame              # Reference frame for the position constraint
        pos_c.link_name = self.tool_frame                    # Target link to apply the position constraint
        pos_c.constraint_region.primitives = [               # Define a small 3D region (box) around the target pose
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.001, 0.001, 0.001])
        ]
        pos_c.constraint_region.primitive_poses = [pose]     # Center the constraint region at the target pose
        pos_c.weight = 1.0                                   # Importance (weight) of this position constraint

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.base_frame              # Reference frame for the orientation constraint
        ori_c.link_name = self.tool_frame                    # Target link to apply the orientation constraint
        ori_c.orientation = pose.orientation                 # Desired orientation for the end-effector
        ori_c.absolute_x_axis_tolerance = 0.1                # Allowed tolerance around X-axis (radians)
        ori_c.absolute_y_axis_tolerance = 0.1                # Allowed tolerance around Y-axis (radians)
        ori_c.absolute_z_axis_tolerance = 0.1                # Allowed tolerance around Z-axis (radians)
        ori_c.weight = 1.0                                   # Importance (weight) of this orientation constraint

        goal = Constraints(
            position_constraints=[pos_c],
            orientation_constraints=[ori_c],
        )
        req.goal_constraints = [goal]

        description = f"Pose move: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})"
        return self.send_goal(req, description, vel_scale, acc_scale)
