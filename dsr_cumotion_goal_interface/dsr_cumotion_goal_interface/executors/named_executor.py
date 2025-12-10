import math
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from .base_executor import MoveItExecutorBase


class NamedExecutor(MoveItExecutorBase):
    """Executor for predefined named poses (e.g., HOME, SET)."""

    # Define named joint targets (degrees)
    NAMED_JOINTS = {
        "HOME": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
        "SET":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

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

    # Main execution entry
    def execute(self, msg, vel_scale=None, acc_scale=None, on_complete=None):
        """Execute motion toward a predefined named pose."""
        name = (msg.name or "").upper().strip()
        if not name:
            self.node.get_logger().error("[NamedExecutor] Named move requires a pose name.")
            if on_complete:
                on_complete(False)
            return False

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
            f"[NamedExecutor] Executing named move '{name}' "
            f"(vel_scale={vel_scale:.2f}, acc_scale={acc_scale:.2f})"
        )

        # Lookup predefined joint angles
        if name not in self.NAMED_JOINTS:
            self.node.get_logger().error(
                f"[NamedExecutor] Unknown named pose '{name}'. "
                f"Define it in NAMED_JOINTS to use it."
            )
            return False

        joints_deg = self.NAMED_JOINTS[name]
        joints_rad = [math.radians(j) for j in joints_deg]

        # Build MotionPlanRequest
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = self.planner_id
        req.allowed_planning_time = float(self.allowed_planning_time)
        req.num_planning_attempts = int(self.num_planning_attempts)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)

        # Add joint constraints
        constraints = Constraints()
        for i, angle in enumerate(joints_rad):
            jc = JointConstraint()
            jc.joint_name = f"joint_{i + 1}"          # Target joint name
            jc.position = angle                       # Desired joint angle (radians)
            jc.tolerance_above = 0.01                 # Allowed deviation above target
            jc.tolerance_below = 0.01                 # Allowed deviation below target
            jc.weight = 1.0                           # Importance (weight) of this constraint
            constraints.joint_constraints.append(jc)

        req.goal_constraints = [constraints]

        # Send goal to MoveGroup Action Server
        description = f"Named move: {name}"
        return self.send_goal(req, description, vel_scale, acc_scale, on_complete=on_complete)
