import math
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from .base_executor import MoveItExecutorBase


def normalize_angle(rad: float) -> float:
    """Normalize an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


class JointExecutor(MoveItExecutorBase):
    """Executor for joint-based motion commands (with start_state support)."""

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
        current_joint_positions=None,
        current_joint_names=None,
    ):
        super().__init__(node, group_name, pipeline_id, base_frame, tool_frame)
        self.planner_id = planner_id
        self.allowed_planning_time = allowed_planning_time
        self.num_planning_attempts = num_planning_attempts
        self.default_vel_scale = default_vel_scale
        self.default_acc_scale = default_acc_scale

        # Initialize current joint state
        self.current_joint_positions = current_joint_positions or []
        self.current_joint_names = current_joint_names or []
        self.current_state = RobotState()
        self.current_joint_state = JointState()
        self.current_joint_state.name = self.current_joint_names
        self.current_joint_state.position = self.current_joint_positions
        self.current_state.joint_state = self.current_joint_state

    def execute(self, msg, vel_scale=None, acc_scale=None):
        """Convert a joint-type TargetPose message into a MotionPlanRequest and send goal."""

        # Validate joint input
        if not hasattr(msg, "joints") or len(msg.joints) != 6:
            self.node.get_logger().error("Expected 6 joint values (degrees)")
            return False

        # Convert degrees to radians and normalize
        joints_rad = [math.radians(j) for j in msg.joints]
        joints_rad = [
            normalize_angle(r) if abs(r) <= math.pi * 1.1 else r for r in joints_rad
        ]

        # Determine velocity and acceleration scaling
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
            f"[JointExecutor] Executing joint move "
            f"(vel_scale={vel_scale:.2f}, acc_scale={acc_scale:.2f})"
        )

        # Build MotionPlanRequest
        req = MotionPlanRequest()
        req.start_state = self.current_state  # Include current joint state as the starting point
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = self.planner_id
        req.allowed_planning_time = float(self.allowed_planning_time)
        req.num_planning_attempts = int(self.num_planning_attempts)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)

        # Define goal constraints for target joint positions
        constraints = Constraints()
        for i, angle in enumerate(joints_rad):
            jc = JointConstraint()
            jc.joint_name = f"joint_{i + 1}"      # Target joint name
            jc.position = angle                   # Desired joint position (radians)
            jc.tolerance_above = 0.01             # Allowable deviation above target
            jc.tolerance_below = 0.01             # Allowable deviation below target
            jc.weight = 1.0                       # Importance weight for this joint
            constraints.joint_constraints.append(jc)

        req.goal_constraints = [constraints]

        # Build a short description for logging
        description = f"Joint move: {[round(math.degrees(a), 1) for a in joints_rad]}"
        return self.send_goal(req, description, vel_scale, acc_scale)
