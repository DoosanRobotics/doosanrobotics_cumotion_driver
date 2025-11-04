#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from dsr_cumotion.msg import TargetPose
import time


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        self.pub = self.create_publisher(TargetPose, '/target_pose', 10)
        self.targets = [
            (0.4, 0.0, 0.3, 1),
            (0.5, 0.8, 0.4, 2),
            (0.5, -0.1, 0.4, 3),
            (0.4, 0.0, 0.2, 5),
        ]
        self.get_logger().info("Publishing target poses to /target_pose...")
        self.publish_targets()

    def publish_targets(self):
        msg = TargetPose()
        for (x, y, z, ori_id) in self.targets:
            msg.x = x
            msg.y = y
            msg.z = z
            msg.orientation_id = ori_id
            self.pub.publish(msg)
            self.get_logger().info(f"Sent TargetPose → x={x:.2f}, y={y:.2f}, z={z:.2f}, id={ori_id}")
            time.sleep(5.0)
        self.get_logger().info("All target poses sent. Node will exit cleanly.")


def main(args=None):
    rclpy.init(args=args)
    node = TargetPosePublisher()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
