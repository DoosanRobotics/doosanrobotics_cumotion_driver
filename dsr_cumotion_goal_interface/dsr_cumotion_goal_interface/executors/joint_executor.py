import math
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from .base_executor import MoveItExecutorBase


class JointExecutor(MoveItExecutorBase):
    """Executor for joint-based motion commands (simplified & updated, with start_state support)."""

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

        # Subscribe to joint states to build explicit start_state
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

    # Main execution entry
    def execute(self, msg, vel_scale=None, acc_scale=None):
        """Convert TargetPose (joint type) into MotionPlanRequest and send goal."""

        # Validate joint input
        if not hasattr(msg, "joints") or len(msg.joints) != 6:
            self.node.get_logger().error("Expected 6 joint values (degrees)")
            return False

        # Convert from degrees to radians
        joints_rad = [math.radians(j) for j in msg.joints]

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
            f"[JointExecutor] Executing joint move "
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

        # Explicitly set start_state
        if self.latest_joint_state:
            start_state = RobotState()
            start_state.joint_state = self.latest_joint_state
            req.start_state = start_state
            self.node.get_logger().info(
                f"[JointExecutor] Using explicit start_state with {len(self.latest_joint_state.name)} joints."
            )
        else:
            self.node.get_logger().warn(
                "[JointExecutor] No joint_state received yet — using MoveIt default start state."
            )

        # Add joint constraints
        constraints = Constraints()
        for i, angle in enumerate(joints_rad):
            jc = JointConstraint()
            jc.joint_name = f"joint_{i + 1}"          # Target joint name
            jc.position = angle                       # Desired joint position (radians)
            jc.tolerance_above = 0.01                 # Allowable deviation above target
            jc.tolerance_below = 0.01                 # Allowable deviation below target
            jc.weight = 1.0                           # Importance of this joint constraint
            constraints.joint_constraints.append(jc)

        req.goal_constraints = [constraints]

        description = f"Joint move: {[round(math.degrees(a), 1) for a in joints_rad]}"
        return self.send_goal(req, description, vel_scale, acc_scale)
