#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dynamic Collision Manager for MoveIt + cuMotion
-------------------------------------------------
- Loads collision objects from YAML (via --ros-args -p config_file:=...)
- Publishes once on startup
- Subscribes to /collision_remove (std_msgs/String):
    "box1,box2"  -> remove specific objects
    ""            -> remove all objects
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class ObstacleManager(Node):
    def __init__(self):
        super().__init__("obstacle_manager")

        # ---- Load config file ----
        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value

        if config_file and os.path.exists(config_file):
            self.get_logger().info(f"Loading collision objects from: {config_file}")
            with open(config_file, "r") as f:
                self.objects = yaml.safe_load(f)
            # normalize format
            if isinstance(self.objects, dict) and "objects" in self.objects:
                self.objects = self.objects["objects"]
            elif not isinstance(self.objects, list):
                self.get_logger().warn("Invalid YAML structure. Expected list under key 'objects'.")
                self.objects = []
        else:
            self.get_logger().warn("No valid config_file provided or file not found.")
            self.objects = []

        # ---- ROS setup ----
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.remove_sub = self.create_subscription(String, "/collision_remove", self.remove_callback, 10)

        self.add_collision_objects()

    # ---- Add objects ----
    def add_collision_objects(self):
        scene = PlanningScene()
        scene.is_diff = True

        for obj_def in self.objects:
            try:
                object_id = obj_def["id"]
                object_type = obj_def["type"].upper()
                position = obj_def["position"]
                dimensions = obj_def["dimensions"]
            except KeyError as e:
                self.get_logger().error(f"Missing key {e} in object definition: {obj_def}")
                continue

            obj = CollisionObject()
            obj.header.frame_id = "base_link"
            obj.id = object_id
            primitive = SolidPrimitive()

            if object_type == "BOX":
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = dimensions
            elif object_type == "SPHERE":
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [dimensions[0]]  # radius
            elif object_type == "CYLINDER":
                primitive.type = SolidPrimitive.CYLINDER
                if len(dimensions) == 2:
                    # Assume YAML gives [radius, height]
                    primitive.dimensions = [dimensions[1], dimensions[0]]
                else:
                    self.get_logger().warn(f"CYLINDER '{object_id}' expects [radius, height], got {dimensions}")
                    continue
            else:
                self.get_logger().error(f"Unsupported object type: {object_type}")
                continue

            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.w = 1.0

            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
            obj.operation = CollisionObject.ADD

            scene.world.collision_objects.append(obj)
            self.get_logger().info(f"Added {object_type} '{object_id}' at {position} with dims {dimensions}")

        if scene.world.collision_objects:
            self.publish_scene(scene)
        else:
            self.get_logger().warn("No valid collision objects to publish.")

    # ---- Remove objects ----
    def remove_callback(self, msg: String):
        data = msg.data.strip()
        scene = PlanningScene()
        scene.is_diff = True

        if not data:
            self.get_logger().warn("Removing ALL collision objects from scene!")
            # Removing all known objects
            for obj_def in self.objects:
                co = CollisionObject()
                co.id = obj_def["id"]
                co.header.frame_id = "base_link"
                co.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(co)
        else:
            ids = [s.strip() for s in data.split(",") if s.strip()]
            for obj_id in ids:
                co = CollisionObject()
                co.id = obj_id
                co.header.frame_id = "base_link"
                co.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(co)
                self.get_logger().info(f"Requested removal of '{obj_id}'")

        self.publish_scene(scene)

    # ---- Publish helper ----
    def publish_scene(self, scene: PlanningScene):
        self.scene_pub.publish(scene)
        self.get_logger().info(f"PlanningScene diff published ({len(scene.world.collision_objects)} objects).")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
