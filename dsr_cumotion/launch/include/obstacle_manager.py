#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dynamic Collision Manager for MoveIt + cuMotion
-------------------------------------------------
- Loads collision objects from YAML (via --ros-args -p config_file:=...)
- Supports BOX, SPHERE, CYLINDER, MESH
- Publishes ONCE on startup (after small delay for MoveIt sync)
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
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import trimesh  # pip install trimesh


class ObstacleManager(Node):
    def __init__(self):
        super().__init__("obstacle_manager")

        # ---- Load config file ----
        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value
        self.config_dir = ""

        if config_file and os.path.exists(config_file):
            self.config_dir = os.path.dirname(os.path.abspath(config_file))
            self.get_logger().info(f"Loading collision objects from: {config_file}")
            with open(config_file, "r") as f:
                self.objects = yaml.safe_load(f)
            if isinstance(self.objects, dict) and "objects" in self.objects:
                self.objects = self.objects["objects"]
            elif not isinstance(self.objects, list):
                self.get_logger().warn("Invalid YAML structure. Expected list under key 'objects'.")
                self.objects = []
        else:
            self.get_logger().warn("No valid config_file provided or file not found.")
            self.objects = []

        # ---- ROS setup ----
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", qos)
        self.remove_sub = self.create_subscription(String, "/collision_remove", self.remove_callback, 10)

        # Publish once after short delay (to let MoveIt initialize)
        self.timer_once = self.create_timer(2.0, self._once_add_objects)

    # ---- Timer callback for one-time publish ----
    def _once_add_objects(self):
        self.add_collision_objects()
        self.timer_once.cancel()  # stop the timer after first publish

    # ---- Add objects ----
    def add_collision_objects(self):
        scene = PlanningScene()
        now = self.get_clock().now().to_msg()
        scene.is_diff = True

        for obj_def in self.objects:
            try:
                object_id = obj_def["id"]
                object_type = obj_def["type"].upper()
                position = obj_def.get("position", [0.0, 0.0, 0.0])
            except KeyError as e:
                self.get_logger().error(f"Missing key {e} in object definition: {obj_def}")
                continue

            obj = CollisionObject()
            obj.header.frame_id = obj_def.get("frame_id", "base_link")
            obj.header.stamp = now
            obj.id = object_id

            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]

            orientation = obj_def.get("orientation", [0.0, 0.0, 0.0, 1.0])
            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]

            # --- Type handling ---
            if object_type == "BOX":
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = obj_def["dimensions"]
                obj.primitives = [primitive]
                obj.primitive_poses = [pose]

            elif object_type == "SPHERE":
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [obj_def["dimensions"][0]]
                obj.primitives = [primitive]
                obj.primitive_poses = [pose]

            elif object_type == "CYLINDER":
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.CYLINDER
                dims = obj_def["dimensions"]
                if len(dims) == 2:
                    primitive.dimensions = [dims[1], dims[0]]  # height, radius
                else:
                    self.get_logger().warn(f"CYLINDER '{object_id}' expects [radius, height], got {dims}")
                    continue
                obj.primitives = [primitive]
                obj.primitive_poses = [pose]

            elif object_type == "MESH":
                mesh_path = obj_def.get("mesh_path", obj_def.get("mesh_resource", ""))
                scale = obj_def.get("scale", [1.0, 1.0, 1.0])

                if mesh_path and not os.path.isabs(mesh_path):
                    mesh_path = os.path.join(self.config_dir, mesh_path)
                    mesh_path = os.path.normpath(mesh_path)

                if not mesh_path or not os.path.exists(mesh_path):
                    self.get_logger().error(f"MESH '{object_id}' missing or invalid mesh_path: {mesh_path}")
                    continue

                try:
                    mesh_msg = self.load_mesh(mesh_path, scale)
                    obj.meshes = [mesh_msg]
                    obj.mesh_poses = [pose]
                    self.get_logger().info(f"Loaded MESH '{object_id}' from {mesh_path} with scale {scale}")
                except Exception as e:
                    self.get_logger().error(f"Failed to load mesh '{mesh_path}': {e}")
                    continue

            else:
                self.get_logger().error(f"Unsupported object type: {object_type}")
                continue

            obj.operation = CollisionObject.ADD
            scene.world.collision_objects.append(obj)

        # Publish once
        if scene.world.collision_objects:
            self.scene_pub.publish(scene)
            self.get_logger().info(f" Published {len(scene.world.collision_objects)} collision objects to planning scene.")
        else:
            self.get_logger().warn("No valid collision objects to publish.")

    # ---- Load mesh helper ----
    def load_mesh(self, filepath, scale):
        """Convert .stl/.obj/.dae to shape_msgs/Mesh"""
        mesh = trimesh.load_mesh(filepath)
        if mesh.is_empty:
            raise ValueError("Empty mesh file.")

        mesh.apply_scale(scale)

        mesh_msg = Mesh()
        for tri in mesh.faces:
            tri_msg = MeshTriangle()
            tri_msg.vertex_indices = [int(i) for i in tri]
            mesh_msg.triangles.append(tri_msg)

        for vertex in mesh.vertices:
            point = Pose().position.__class__()  # geometry_msgs/Point
            point.x, point.y, point.z = vertex
            mesh_msg.vertices.append(point)

        return mesh_msg

    # Remove objects
    def remove_callback(self, msg: String):
        data = msg.data.strip()
        scene = PlanningScene()
        scene.is_diff = True
        now = self.get_clock().now().to_msg()

        if not data:
            self.get_logger().warn("Removing ALL collision objects from scene!")
            for obj_def in self.objects:
                co = CollisionObject()
                co.id = obj_def["id"]
                co.header.frame_id = obj_def.get("frame_id", "base_link")
                co.header.stamp = now
                co.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(co)
        else:
            ids = [s.strip() for s in data.split(",") if s.strip()]
            for obj_id in ids:
                co = CollisionObject()
                co.id = obj_id
                # try to preserve frame_id from the configured objects
                frame = "base_link"
                for o in self.objects:
                    if o.get("id") == obj_id:
                        frame = o.get("frame_id", "base_link")
                        break
                co.header.frame_id = frame
                co.header.stamp = now
                co.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(co)
                self.get_logger().info(f"Requested removal of '{obj_id}'")

        self.scene_pub.publish(scene)
        self.get_logger().info(f"Removed {len(scene.world.collision_objects)} object(s) from scene.")


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
