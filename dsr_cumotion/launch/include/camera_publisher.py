#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('static_depth_camera_node')
    
        # Publishers: send both camera calibration info and depth image frames
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.depth_pub = self.create_publisher(Image, '/depth', 10)

        # Timer: publishes data periodically (10 Hz = every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.publish_topics)

        # Camera intrinsic parameters
        # These are the same as a RealSense D series camera with 1280x720 resolution.
        # fx, fy: focal lengths (in pixels)
        # cx, cy: principal point coordinates (image center)
        self.width = 1280
        self.height = 720
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 640.0
        self.cy = 360.0

        # Depth image parameters
        # depth_value_m : the constant distance (in meters) that every pixel represents.
        #                 For example, 0.4 means the entire image corresponds to a flat surface
        #                 0.4 m away from the camera optical center.
        # depth_encoding : '32FC1' = 32-bit float, single channel, distance in meters
        # frame_id : coordinate frame associated with the depth data (here, the robot's tool frame)
        self.depth_value_m = 0.4
        self.depth_encoding = '32FC1'
        self.frame_id = 'tool0'

        self.get_logger().info("Static depth and camera_info publishers started (frame_id=tool0).")

    def publish_topics(self):

        # Common message header
        # Includes timestamp and frame ID ("tool0" frame).
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # current ROS time
        header.frame_id = self.frame_id

        # 1. CameraInfo message
        # Describes the virtual camera's intrinsic calibration parameters.
        info = CameraInfo()
        info.header = header
        info.height = self.height
        info.width = self.width
        info.distortion_model = "plumb_bob"  # Standard pinhole camera model
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]   # No lens distortion

        # Intrinsic matrix (K)
        info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix (R): identity for a single camera
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix (P)
        # Extends K to 3x4 form for projection from 3D to 2D
        info.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # No binning or region of interest cropping
        info.binning_x = 0
        info.binning_y = 0
        info.roi.do_rectify = False

        # 2. Depth Image message
        # Contains a 2D image where each pixel value = constant depth_value_m.
        # This simulates a perfectly flat surface at a fixed distance.
        # Create a numpy array filled with a constant float32 value
        depth_img = np.full((self.height, self.width), self.depth_value_m, dtype=np.float32)

        # Populate ROS2 Image message fields
        depth_msg = Image()
        depth_msg.header = header
        depth_msg.height = self.height
        depth_msg.width = self.width
        depth_msg.encoding = self.depth_encoding     # "32FC1" = float32, 1 channel
        depth_msg.is_bigendian = 0                   # little-endian (standard)
        depth_msg.step = self.width * 4              # 4 bytes per pixel (float32)
        depth_msg.data = depth_img.tobytes()         # convert numpy array to raw bytes

        # Publish both messages
        self.info_pub.publish(info)
        self.depth_pub.publish(depth_msg)


def main(args=None):
    """Initialize and spin the node."""
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
