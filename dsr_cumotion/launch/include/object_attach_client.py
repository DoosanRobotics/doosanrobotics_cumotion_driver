#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from isaac_ros_cumotion_interfaces.action import AttachObject
from std_msgs.msg import Header
import time


class ObjectAttachClient(Node):
    """Action client that sends an AttachObject goal to the CuMotion object attachment server."""

    def __init__(self):
        super().__init__('object_attach_client')
        self._client = ActionClient(self, AttachObject, '/attach_object')
        self.send_goal_once_ready()

    def send_goal_once_ready(self):
        """Wait until the AttachObject action server is ready, then send a single goal."""
        max_wait_sec = 15.0  # maximum wait time
        start_time = time.time()

        self.get_logger().info('Waiting for /attach_object action server...')
        while not self._client.wait_for_server(timeout_sec=1.0):
            if time.time() - start_time > max_wait_sec:
                self.get_logger().error('Timeout: AttachObject action server not available!')
                rclpy.shutdown()
                return
            self.get_logger().info('  ...still waiting...')

        # Once server is ready, send goal
        self.get_logger().info('AttachObject server ready! Sending goal...')
        self.send_goal()

    def send_goal(self):
        """Build and send the AttachObject goal message."""
        goal = AttachObject.Goal()
        goal.attach_object = True
        goal.fallback_radius = 0.05

        goal.object_config.header = Header(frame_id='grasp_frame')
        goal.object_config.type = 10
        goal.object_config.mesh_resource = (
            '/ros2_ws/src/cumotion/dsr_cumotion/meshes/object/box_7.obj'
        )

        goal.object_config.pose.position.x = 0.0
        goal.object_config.pose.position.y = 0.0
        goal.object_config.pose.position.z = 0.05
        goal.object_config.pose.orientation.x = 1.0
        goal.object_config.pose.orientation.y = 0.0
        goal.object_config.pose.orientation.z = 0.0
        goal.object_config.pose.orientation.w = 0.0

        goal.object_config.scale.x = 1.0
        goal.object_config.scale.y = 1.0
        goal.object_config.scale.z = 1.0

        self.get_logger().info('Sending AttachObject goal...')
        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('AttachObject goal rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'AttachObject result: {result}')
        self.get_logger().info('AttachObject completed successfully.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAttachClient()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
