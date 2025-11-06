#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Import separated message types
from dsr_cumotion_msgs.msg import (
    TargetPose,
    TargetJoint,
    TargetNamed,
    TargetRelative,
)

# Import executor classes
from dsr_cumotion_goal_interface.executors.pose_executor import PoseExecutor
from dsr_cumotion_goal_interface.executors.joint_executor import JointExecutor
from dsr_cumotion_goal_interface.executors.named_executor import NamedExecutor
from dsr_cumotion_goal_interface.executors.relative_executor import RelativeExecutor


class MoveCommandNode(Node):
    """MoveCommandNode — handles four motion types with dedicated topics and executors."""

    def __init__(self):
        super().__init__("move_command_node")

        # Declare configurable parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("planning_group", "manipulator"),
                ("planner_pipeline", "isaac_ros_cumotion"),
                ("planner_id", "cuMotion"),
                ("base_frame", "base_link"),
                ("tool_frame", "grasp_frame"),
                ("allowed_planning_time", 5.0),
                ("num_planning_attempts", 10),
                ("default_max_vel_scale", 1.0),
                ("default_max_acc_scale", 1.0),
            ],
        )

        # Read parameter values
        group_name = self.get_parameter("planning_group").value
        pipeline_id = self.get_parameter("planner_pipeline").value
        planner_id = self.get_parameter("planner_id").value
        base_frame = self.get_parameter("base_frame").value
        tool_frame = self.get_parameter("tool_frame").value
        allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        default_vel_scale = float(self.get_parameter("default_max_vel_scale").value)
        default_acc_scale = float(self.get_parameter("default_max_acc_scale").value)

        self.get_logger().info(
            f"[MoveCommandNode] group={group_name}, planner={planner_id}, "
            f"pipeline={pipeline_id}, base={base_frame}, tool={tool_frame}, "
            f"vel_scale={default_vel_scale}, acc_scale={default_acc_scale}"
        )

        # Initialize executors for each motion type
        self.executors = {
            "pose": PoseExecutor(
                self, group_name, pipeline_id, base_frame, tool_frame,
                planner_id=planner_id,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                default_vel_scale=default_vel_scale,
                default_acc_scale=default_acc_scale,
            ),
            "joint": JointExecutor(
                self, group_name, pipeline_id, base_frame, tool_frame,
                planner_id=planner_id,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                default_vel_scale=default_vel_scale,
                default_acc_scale=default_acc_scale,
            ),
            "named": NamedExecutor(
                self, group_name, pipeline_id, base_frame, tool_frame,
                planner_id=planner_id,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                default_vel_scale=default_vel_scale,
                default_acc_scale=default_acc_scale,
            ),
            "relative": RelativeExecutor(
                self, group_name, pipeline_id, base_frame, tool_frame,
                planner_id=planner_id,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                default_vel_scale=default_vel_scale,
                default_acc_scale=default_acc_scale,
            ),
        }

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self.sub_pose = self.create_subscription(
            TargetPose, "/target_pose", self.pose_callback, qos_profile
        )
        self.sub_joint = self.create_subscription(
            TargetJoint, "/target_joint", self.joint_callback, qos_profile
        )
        self.sub_named = self.create_subscription(
            TargetNamed, "/target_named", self.named_callback, qos_profile
        )
        self.sub_relative = self.create_subscription(
            TargetRelative, "/target_relative", self.relative_callback, qos_profile
        )

        self.get_logger().info(
            "[MoveCommandNode] Subscribed to: "
            "/target_pose, /target_joint, /target_named, /target_relative"
        )


    def pose_callback(self, msg: TargetPose):
        """Execute absolute pose motion."""
        self._execute("pose", msg, msg.max_vel_scale, msg.max_acc_scale)

    def joint_callback(self, msg: TargetJoint):
        """Execute joint-space motion."""
        self._execute("joint", msg, msg.max_vel_scale, msg.max_acc_scale)

    def named_callback(self, msg: TargetNamed):
        """Execute named target motion."""
        self._execute("named", msg, msg.max_vel_scale, msg.max_acc_scale)

    def relative_callback(self, msg: TargetRelative):
        """Execute relative motion."""
        self._execute("relative", msg, msg.max_vel_scale, msg.max_acc_scale)


    def _execute(self, mode: str, msg, vel_scale: float, acc_scale: float):
        """Common execution handler for all motion types."""
        if mode not in self.executors:
            self.get_logger().error(f"Invalid motion type: {mode}")
            return

        executor = self.executors[mode]
        vel_scale = vel_scale if vel_scale > 0.0 else executor.default_vel_scale
        acc_scale = acc_scale if acc_scale > 0.0 else executor.default_acc_scale

        self.get_logger().info(
            f"[Command] Executing '{mode}' "
            f"(vel_scale={vel_scale:.2f}, acc_scale={acc_scale:.2f})"
        )

        executor.execute(msg, vel_scale=vel_scale, acc_scale=acc_scale)
        self.get_logger().info(f"[Command] '{mode}' execution completed.")


def main(args=None):
    rclpy.init(args=args)
    node = MoveCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MoveCommandNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
