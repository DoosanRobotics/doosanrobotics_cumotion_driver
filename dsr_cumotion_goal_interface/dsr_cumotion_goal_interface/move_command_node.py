#!/usr/bin/env python3
import threading
import queue
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from dsr_cumotion_msgs.msg import TargetPose
from dsr_cumotion_goal_interface.executors.pose_executor import PoseExecutor
from dsr_cumotion_goal_interface.executors.joint_executor import JointExecutor
from dsr_cumotion_goal_interface.executors.named_executor import NamedExecutor
from dsr_cumotion_goal_interface.executors.relative_executor import RelativeExecutor


class MoveCommandNode(Node):
    """Main node that subscribes to /target_pose and executes queued motion commands sequentially"""

    def __init__(self):
        super().__init__("move_command_node")

        self.cb_group = ReentrantCallbackGroup()
        self.cmd_queue = queue.Queue()
        self.executor_thread = threading.Thread(target=self._process_queue, daemon=True)
        self.executor_thread.start()

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

        self.subscription = self.create_subscription(
            TargetPose, "/target_pose", self.command_callback, 10, callback_group=self.cb_group
        )
        self.get_logger().info("[MoveCommandNode] Listening to /target_pose...")

    def command_callback(self, msg: TargetPose):
        move_type = (msg.move_type or "").lower().strip()
        if move_type not in self.executors:
            self.get_logger().error(f"Invalid move_type: {move_type}")
            return

        self.cmd_queue.put(msg)
        self.get_logger().info(
            f"[Dispatcher] Queued move_type='{move_type}' "
            f"(vel_scale={msg.max_vel_scale:.2f}, acc_scale={msg.max_acc_scale:.2f}) "
            f"(queue size={self.cmd_queue.qsize()})"
        )

    def _process_queue(self):
        while True:
            msg = self.cmd_queue.get()
            move_type = msg.move_type.lower().strip()
            executor = self.executors[move_type]

            vel_scale = msg.max_vel_scale if msg.max_vel_scale > 0.0 else executor.default_vel_scale
            acc_scale = msg.max_acc_scale if msg.max_acc_scale > 0.0 else executor.default_acc_scale

            self.get_logger().info(
                f"[ExecutorQueue] Executing {move_type} with vel_scale={vel_scale}, acc_scale={acc_scale}"
            )

            executor.execute(msg, vel_scale=vel_scale, acc_scale=acc_scale)

            while executor.current_goal_handle is not None:
                rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info(f"[ExecutorQueue] Done: {move_type}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveCommandNode()
    executor = MultiThreadedExecutor()
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
