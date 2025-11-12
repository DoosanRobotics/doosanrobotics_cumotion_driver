#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from dsr_cumotion_msgs.srv import PickPlace
from dsr_cumotion_msgs.msg import TargetPose
from isaac_ros_cumotion_interfaces.action import AttachObject
from isaac_manipulator_ros_python_utils.types import AttachState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3
from dsr_cumotion_goal_interface.executors.relative_executor import RelativeExecutor
import time

class PickPlaceServer(Node):
    """Pick & Place – Async Flow + AttachObject integration"""

    def __init__(self):
        super().__init__("pick_and_place_server")
        self.cb_group = ReentrantCallbackGroup()

        # Motion executor (MoveIt + cuMotion)
        self.relative_executor = RelativeExecutor(
            self,
            group_name="manipulator",
            pipeline_id="isaac_ros_cumotion",
            base_frame="base_link",
            tool_frame="grasp_frame",
            planner_id="cuMotion",
            allowed_planning_time=5.0,
            num_planning_attempts=10,
            default_vel_scale=1.0,
            default_acc_scale=1.0,
        )

        # AttachObject action client
        self.attach_ac = ActionClient(self, AttachObject, "attach_object", callback_group=self.cb_group)

        # Default mesh
        self.default_mesh_path = "/ros2_ws/src/cumotion/dsr_cumotion/meshes/object/box_7.obj"

        # PickPlace service
        self.srv = self.create_service(
            PickPlace, "pick_place_command", self.handle_request, callback_group=self.cb_group
        )

        self.current_mode = None
        self.request_data = None
        self.get_logger().info(" PickPlaceServer ready (AttachObject integrated).")

    # Service entry
    def handle_request(self, req: PickPlace.Request, res: PickPlace.Response):
        """Start async pick/place sequence"""
        self.current_mode = req.mode
        self.request_data = req

        dx, dy, dz = req.dx, req.dy, req.dz
        drx, dry, drz = req.drx, req.dry, req.drz
        vel, acc = req.vel, req.acc

        self.get_logger().info(
            f"[PickPlaceServer] mode={req.mode}, Δ=({dx:.3f},{dy:.3f},{dz:.3f}), "
            f"rot=({drx:.1f},{dry:.1f},{drz:.1f}), v={vel}, a={acc}"
        )

        self._send_relative(dx, dy, dz, drx, dry, drz, vel, acc, "descend", on_complete=self._on_descend_done)

        res.success = True
        res.message = "Pick/place sequence started (async)."
        return res

    # Async callbacks
    def _on_descend_done(self, success):
        if not success:
            self.get_logger().warn("[PickPlaceServer] Descend failed.")
            return
        time.sleep(1.0)
        if self.current_mode == 0:
            self.get_logger().info("[PickPlaceServer] Descend complete → Attach object.")
            self._attach_object_async(True, on_complete=self._on_attach_done)
        else:
            self.get_logger().info("[PickPlaceServer] Descend complete → Detach object.")
            self._attach_object_async(False, on_complete=self._on_detach_done)
        time.sleep(1.0)

    # if you want remove ascend move, self._ascend() to self._on_all_done(True)

    def _on_attach_done(self, success):
        if not success:
            self.get_logger().warn("[PickPlaceServer] Attach failed.")
            return
        self.get_logger().info("[PickPlaceServer] Attach complete → Ascending.")
        self._ascend()
        # self._on_all_done(True)

    def _on_detach_done(self, success):
        if not success:
            self.get_logger().warn("[PickPlaceServer] Detach failed.")
            return
        self.get_logger().info("[PickPlaceServer] Detach complete → Ascending.")
        self._ascend()
        # self._on_all_done(True)

    def _on_all_done(self, success):
        if success:
            self.get_logger().info("[PickPlaceServer] Pick/place sequence completed.")
        else:
            self.get_logger().warn("[PickPlaceServer] Sequence finished with errors.")
        self.current_mode = None
        self.request_data = None

    # Relative move helpers
    def _send_relative(self, dx, dy, dz, drx, dry, drz, vel, acc, label, on_complete):
        msg = TargetPose()
        msg.move_type = "relative"
        msg.dx, msg.dy, msg.dz = dx, dy, dz
        msg.drx, msg.dry, msg.drz = drx, dry, drz
        msg.max_vel_scale = vel
        msg.max_acc_scale = acc
        desc = f"{label}: Δ=({dx:.3f},{dy:.3f},{dz:.3f}), rot=({drx:.1f},{dry:.1f},{drz:.1f})"
        self.relative_executor.execute(msg, vel_scale=vel, acc_scale=acc, on_complete=on_complete)

    def _ascend(self):
        req = self.request_data
        dx, dy, dz = -req.dx, -req.dy, -req.dz
        drx, dry, drz = -req.drx, -req.dry, -req.drz
        vel, acc = req.vel, req.acc
        self._send_relative(dx, dy, dz, drx, dry, drz, vel, acc, "ascend", on_complete=self._on_all_done)

    # AttachObject integration 
    def _attach_object_async(self, attach: bool, on_complete):
        if not self.attach_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("AttachObject action server not available.")
            on_complete(False)
            return

        goal = AttachObject.Goal()
        goal.attach_object = AttachState.ATTACH.value if attach else AttachState.DETACH.value
        goal.fallback_radius = 0.15
        goal.object_config = self._make_marker("grasp_frame", self.default_mesh_path)
        action_type = "ATTACH" if attach else "DETACH"
        self.get_logger().info(f"[{action_type}] Sending goal to AttachObject...")

        # send goal async + result callback
        future = self.attach_ac.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_sent(f, action_type, on_complete))

    def _on_goal_sent(self, future, action_type, on_complete):
        try:
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f"[{action_type}] Goal rejected.")
                on_complete(False)
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self._on_attach_result(f, action_type, on_complete))
        except Exception as e:
            self.get_logger().error(f"[{action_type}] Goal error: {e}")
            on_complete(False)

    def _on_attach_result(self, future, action_type, on_complete):
        try:
            result = future.result().result
            outcome = getattr(result, "outcome", None)
            self.get_logger().info(f"[{action_type}] Result outcome: {outcome}")
            on_complete(True)
        except Exception as e:
            self.get_logger().error(f"[{action_type}] Result error: {e}")
            on_complete(False)

    # Marker for object_config
    def _make_marker(self, frame_id, mesh_path):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = mesh_path
        marker.pose = Pose()
        marker.pose.position.z = 0.035
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
        marker.color.g = 1.0
        marker.color.a = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PickPlaceServer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
