#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from dsr_cumotion_msgs.msg import TargetPose
from dsr_cumotion_goal_interface.executors.pose_executor import PoseExecutor
from dsr_cumotion_goal_interface.executors.joint_executor import JointExecutor
from dsr_cumotion_goal_interface.executors.named_executor import NamedExecutor
from dsr_cumotion_goal_interface.executors.relative_executor import RelativeExecutor


class MoveCommandNode(Node):
    """Simplified MoveCommandNode — single-threaded, clean shutdown, direct motion execution."""

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
                ("retry_num", 0),
            ],
        )

        group_name = self.get_parameter("planning_group").value
        pipeline_id = self.get_parameter("planner_pipeline").value
        planner_id = self.get_parameter("planner_id").value
        base_frame = self.get_parameter("base_frame").value
        tool_frame = self.get_parameter("tool_frame").value
        allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        default_vel_scale = float(self.get_parameter("default_max_vel_scale").value)
        default_acc_scale = float(self.get_parameter("default_max_acc_scale").value)
        retry_count = int(self.get_parameter("retry_num").value)

        self.get_logger().info(
            f"[MoveCommandNode] group={group_name}, planner={planner_id}, "
            f"pipeline={pipeline_id}, base={base_frame}, tool={tool_frame}, "
            f"vel_scale={default_vel_scale}, acc_scale={default_acc_scale}"
        )

        # Initialize executors for different move types
        self.executors = {
            "pose": PoseExecutor(
                self, group_name, pipeline_id, base_frame, tool_frame,
                planner_id=planner_id,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                default_vel_scale=default_vel_scale,
                default_acc_scale=default_acc_scale,
                # retry=retry_count, 
            ),
            "joint": JointExecutor(
                self, group_name, pipeline_id, base_frame, tool_frame,
                planner_id=planner_id,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                default_vel_scale=default_vel_scale,
                default_acc_scale=default_acc_scale,
                # retry=retry_count,
            ),
            "named": NamedExecutor(
                self, group_name, pipeline_id, base_frame, tool_frame,
                planner_id=planner_id,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                default_vel_scale=default_vel_scale,
                default_acc_scale=default_acc_scale,
                # retry=retry_count,
            ),
            "relative": RelativeExecutor(
                self, group_name, pipeline_id, base_frame, tool_frame,
                planner_id=planner_id,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                default_vel_scale=default_vel_scale,
                default_acc_scale=default_acc_scale,
                # retry=retry_count,
            ),
        }

        # Reliable QoS ensures guaranteed message delivery
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # Simple subscription without callback group
        self.subscription = self.create_subscription(
            TargetPose,
            "/target_pose",
            self.command_callback,
            qos_profile,
        )

        self.get_logger().info("[MoveCommandNode] Listening to /target_pose...")

    def destroy_node(self):
        """Clean shutdown: cancel all active goals."""
        self.get_logger().info("[MoveCommandNode] Canceling all active goals...")
        for executor in self.executors.values():
            executor.cancel_current_goal()
        super().destroy_node()

    # Callback: Executes corresponding executor based on move_type
    def command_callback(self, msg: TargetPose):
        move_type = (msg.move_type or "").lower().strip()

        if move_type not in self.executors:
            self.get_logger().error(f"Invalid move_type: {move_type}")
            return

        executor = self.executors[move_type]

        vel_scale = msg.max_vel_scale if msg.max_vel_scale > 0.0 else executor.default_vel_scale
        acc_scale = msg.max_acc_scale if msg.max_acc_scale > 0.0 else executor.default_acc_scale

        self.get_logger().info(
            f"[Command] Executing move_type='{move_type}' "
            f"(vel_scale={vel_scale:.2f}, acc_scale={acc_scale:.2f})"
        )

        try:
            executor.execute(msg, vel_scale=vel_scale, acc_scale=acc_scale)
            self.get_logger().info(f"[Command] Execution request sent for: {move_type}")
        except Exception as e:
            self.get_logger().error(f"[Command] Failed to execute {move_type}: {e}")


# Entry point
def main(args=None):
    rclpy.init(args=args)
    node = MoveCommandNode()
    
    # Use MultiThreadedExecutor to avoid race conditions
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MoveCommandNode...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
