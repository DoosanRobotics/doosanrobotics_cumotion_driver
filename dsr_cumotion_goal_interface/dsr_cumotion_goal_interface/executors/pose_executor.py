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

    # Main execution entry
    def execute(self, msg, vel_scale=None, acc_scale=None, on_complete=None):
        """Build MotionPlanRequest from pose message and send to MoveIt2 (async callback ready)."""

        # Input validation to prevent invalid data from crashing MoveGroup
        if not self._validate_pose_input(msg):
            self.node.get_logger().error("[PoseExecutor] Invalid pose input, aborting.")
            if on_complete:
                on_complete(False)
            return

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

        retry_num = getattr(msg, "retry_num", 0)

        planning_time = float(self.allowed_planning_time)
        attempts = int(self.num_planning_attempts)

        if retry_num > 1:
            planning_time *= retry_num
            attempts = int(attempts * retry_num)

        self.node.get_logger().info(
            f"[PoseExecutor] retry_num={retry_num} → planning_time={planning_time:.3f}, attempts={attempts}"
        )

        # Build MotionPlanRequest
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = self.planner_id

        # 여기서 retry 반영된 값이 들어감
        req.allowed_planning_time = planning_time 
        req.num_planning_attempts = attempts   

        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)

        # Position + Orientation constraints
        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base_frame
        pos_c.link_name = self.tool_frame
        pos_c.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
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

        description = f"Pose move: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}, {msg.rx:.1f}, {msg.ry:.1f}, {msg.rz:.1f})"

        return self.send_goal(
            req,
            description,
            vel_scale,
            acc_scale,
            on_complete=on_complete
        )

    def _validate_pose_input(self, msg) -> bool:
        """Validate pose input to prevent invalid data from crashing MoveGroup."""
        import math
        
        # Check for NaN or Inf in position
        if not all(math.isfinite(v) for v in [msg.x, msg.y, msg.z]):
            self.node.get_logger().error(
                f"[PoseExecutor] Invalid position: x={msg.x}, y={msg.y}, z={msg.z}"
            )
            return False
        
        # Check for reasonable position values (example: within 10m cube)
        if abs(msg.x) > 10.0 or abs(msg.y) > 10.0 or abs(msg.z) > 10.0:
            self.node.get_logger().warn(
                f"[PoseExecutor] Position out of reasonable range: "
                f"x={msg.x}, y={msg.y}, z={msg.z}"
            )
        
        # Check orientation (Euler or Quaternion)
        if hasattr(msg, "rx") and hasattr(msg, "ry") and hasattr(msg, "rz"):
            if not all(math.isfinite(v) for v in [msg.rx, msg.ry, msg.rz]):
                self.node.get_logger().error(
                    f"[PoseExecutor] Invalid Euler angles: "
                    f"rx={msg.rx}, ry={msg.ry}, rz={msg.rz}"
                )
                return False
        
        if hasattr(msg, "qx") and hasattr(msg, "qy") and hasattr(msg, "qz") and hasattr(msg, "qw"):
            if not all(math.isfinite(v) for v in [msg.qx, msg.qy, msg.qz, msg.qw]):
                self.node.get_logger().error(
                    f"[PoseExecutor] Invalid quaternion: "
                    f"qx={msg.qx}, qy={msg.qy}, qz={msg.qz}, qw={msg.qw}"
                )
                return False
            
            # Check if quaternion is normalized (with tolerance)
            quat_norm = math.sqrt(msg.qx**2 + msg.qy**2 + msg.qz**2 + msg.qw**2)
            if abs(quat_norm - 1.0) > 0.1:
                self.node.get_logger().warn(
                    f"[PoseExecutor] Quaternion not normalized: norm={quat_norm:.3f}"
                )
        
        # Check velocity/acceleration scaling
        if hasattr(msg, "max_vel_scale") and msg.max_vel_scale > 0.0:
            if msg.max_vel_scale > 2.0:
                self.node.get_logger().warn(
                    f"[PoseExecutor] Unusually high velocity scale: {msg.max_vel_scale}"
                )
        
        if hasattr(msg, "max_acc_scale") and msg.max_acc_scale > 0.0:
            if msg.max_acc_scale > 2.0:
                self.node.get_logger().warn(
                    f"[PoseExecutor] Unusually high acceleration scale: {msg.max_acc_scale}"
                )
        
        return True
