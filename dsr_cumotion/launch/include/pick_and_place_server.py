#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.task import Future
from dsr_cumotion_msgs.srv import PickPlace
from dsr_cumotion_msgs.msg import TargetPose
from isaac_ros_cumotion_interfaces.action import AttachObject
from isaac_manipulator_ros_python_utils.types import AttachState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3
from dsr_cumotion_goal_interface.executors.relative_executor import RelativeExecutor
import time
class PickPlaceServer(Node):
    """Pick & Place – synchronous service + async motion callbacks"""

    def __init__(self):
        super().__init__("pick_and_place_server")
        self.cb_group = ReentrantCallbackGroup()

        # Motion executor
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
        self.attach_ac = ActionClient(
            self, AttachObject, "attach_object", callback_group=self.cb_group
        )

        # Default mesh
        self.default_mesh_path = "/ros2_ws/src/cumotion/dsr_cumotion/meshes/object/box_7.obj"

        # PickPlace service
        self.srv = self.create_service(
            PickPlace, "pick_place_command", self.handle_request, callback_group=self.cb_group
        )

        self.current_mode = None
        self.request_data = None
        self.result_future = None

        self.get_logger().info("PickPlaceServer ready (synchronous mode).")

    # SERVICE ENTRY POINT (SYNC)
    def handle_request(self, req: PickPlace.Request, res: PickPlace.Response):
        """Wait until pick/place is fully finished."""
        self.get_logger().info(
            f"[SERVICE] PickPlace start: mode={req.mode}, dx={req.dx}, dy={req.dy}, dz={req.dz}"
        )

        self.current_mode = req.mode
        self.request_data = req

        # Future to wait for full sequence completion
        self.result_future = Future()

        # Start DESCEND first
        self._send_relative(
            req.dx, req.dy, req.dz,
            req.drx, req.dry, req.drz,
            req.vel, req.acc,
            label="descend",
            on_complete=self._on_descend_done
        )

        # BLOCK until full sequence is done
        rclpy.spin_until_future_complete(self, self.result_future)

        # Get final result
        success = self.result_future.result()

        res.success = success
        res.message = "Pick/Place done successfully." if success else "Pick/Place failed."

        return res

    # ASYNC CALLBACK FLOW
    def _on_descend_done(self, success):
        if not success:
            self._fail_sequence("Descend failed.")
            return

        self.get_logger().info("[STEP] Descend done.")

        time.sleep(1.0)

        if self.current_mode == 0:  # Pick
            self.get_logger().info("[STEP] Pick mode → ATTACH object.")
            self._attach_object_async(True, on_complete=self._on_attach_done)
        else:  # Place
            self.get_logger().info("[STEP] Place mode → DETACH object.")
            self._attach_object_async(False, on_complete=self._on_detach_done)

    # PICK → Attach → Ascend
    def _on_attach_done(self, success):
        if not success:
            self._fail_sequence("Attach failed.")
            return
        self.get_logger().info("[STEP] Attach done → Ascend")
        # self._ascend()
        self._on_all_done(True)

    # PLACE → Detach → Ascend
    def _on_detach_done(self, success):
        if not success:
            self._fail_sequence("Detach failed.")
            return
        self.get_logger().info("[STEP] Detach done → Ascend")
        # self._ascend()
        self._on_all_done(True)

    # FINAL STEP → COMPLETE
    def _on_all_done(self, success):
        msg = "completed" if success else "failed"
        self.get_logger().info(f"[STEP] Full sequence {msg}.")

        if not self.result_future.done():
            self.result_future.set_result(success)

        self.current_mode = None
        self.request_data = None

    # RELATIVE MOTION helper
    def _send_relative(self, dx, dy, dz, drx, dry, drz, vel, acc, label, on_complete):
        msg = TargetPose()
        msg.move_type = "relative"
        msg.dx, msg.dy, msg.dz = dx, dy, dz
        msg.drx, msg.dry, msg.drz = drx, dry, drz
        msg.max_vel_scale = vel
        msg.max_acc_scale = acc

        self.get_logger().info(
            f"[MOVE] {label}: Δ=({dx:.3f},{dy:.3f},{dz:.3f}), rot=({drx:.2f},{dry:.2f},{drz:.2f})"
        )

        self.relative_executor.execute(
            msg,
            vel_scale=vel,
            acc_scale=acc,
            on_complete=on_complete
        )

    # ASCEND
    def _ascend(self):
        req = self.request_data

        dx, dy, dz = -req.dx, -req.dy, -req.dz
        drx, dry, drz = -req.drx, -req.dry, -req.drz

        self._send_relative(
            dx, dy, dz,
            drx, dry, drz,
            req.vel, req.acc,
            label="ascend",
            on_complete=self._on_all_done
        )

    # ATTACH / DETACH
    def _attach_object_async(self, attach: bool, on_complete):
        if not self.attach_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("AttachObject server unavailable.")
            on_complete(False)
            return

        goal = AttachObject.Goal()
        goal.attach_object = AttachState.ATTACH.value if attach else AttachState.DETACH.value
        goal.fallback_radius = 0.15
        goal.object_config = self._make_marker("grasp_frame", self.default_mesh_path)

        action_name = "ATTACH" if attach else "DETACH"
        self.get_logger().info(f"[ACTION] Sending {action_name} goal...")

        future = self.attach_ac.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_sent(f, action_name, on_complete))

    def _on_goal_sent(self, future, action_name, on_complete):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"[{action_name}] Goal rejected.")
                on_complete(False)
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self._on_action_result(f, action_name, on_complete))
        except Exception as e:
            self.get_logger().error(f"[{action_name}] Goal error: {e}")
            on_complete(False)

    def _on_action_result(self, future, action_name, on_complete):
        try:
            result = future.result().result
            outcome = getattr(result, "outcome", None)
            self.get_logger().info(f"[{action_name}] outcome={outcome}")
            on_complete(True)
        except Exception as e:
            self.get_logger().error(f"[{action_name}] Result error: {e}")
            on_complete(False)

    # util
    def _fail_sequence(self, reason):
        self.get_logger().warn(f"[FAIL] {reason}")
        if self.result_future and not self.result_future.done():
            self.result_future.set_result(False)
        self.current_mode = None
        self.request_data = None

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
