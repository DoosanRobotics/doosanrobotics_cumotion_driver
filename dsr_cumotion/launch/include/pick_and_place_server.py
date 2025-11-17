#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from dsr_cumotion_msgs.srv import PickPlace
from dsr_msgs2.srv import MoveLine
from isaac_ros_cumotion_interfaces.action import AttachObject
from isaac_manipulator_ros_python_utils.types import AttachState

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__("pick_place_server")

        self.cb_group = ReentrantCallbackGroup()  # callback group for async tasks
        self.move_line_cli = self.create_client(MoveLine, "/motion/move_line")  # move_line client
        self.attach_ac = ActionClient(self, AttachObject, "attach_object", callback_group=self.cb_group)  # attach/detach action

        self.srv = self.create_service(
            PickPlace,
            "pick_place_command",
            self.handle_request,
            callback_group=self.cb_group
        )  # main service server

        self.default_mesh_path = "/ros2_ws/src/cumotion/dsr_cumotion/meshes/object/box_7.obj"  # mesh for object

        self.result_future = None       # final result future
        self.current_mode = None        # 0=pick, 1=place
        self.req_data = None            # saved request
        self.motion_args = None         # saved motion parameters

    def handle_request(self, req, res):
        self.current_mode = req.motion_type  # store mode
        self.req_data = req  # store request
        self.result_future = Future()  # future to return result

        self._start_cycle()  # start first step

        rclpy.spin_until_future_complete(self, self.result_future)  # wait until done

        success = self.result_future.result()  # get result
        res.success = success
        res.message = "Success" if success else "Failed"
        return res

    def _start_cycle(self):
        req = self.req_data
        dx = req.dx * 1000.0    # convert to mm
        dy = req.dy * 1000.0
        dz = req.dz * 1000.0

        # store for ascend
        self.motion_args = {
            "dx": dx, "dy": dy, "dz": dz,
            "drx": req.drx, "dry": req.dry, "drz": req.drz,
            "vel": req.vel, "acc": req.acc,
            "ref": req.ref, "mv_mode": req.mv_mode
        }

        # send descend motion
        self._call_move_line(dx, dy, dz, req.drx, req.dry, req.drz,
                             req.vel, req.acc, req.ref, req.mv_mode,
                             self._on_descend_done)

        # if self.current_mode == 0:
        #     self._attach_object_async(True, self._finish)
        # else:
        #     self._attach_object_async(False, self._finish)
            
    def _on_descend_done(self, ok):
        if not ok:
            self._finish(False)  # stop if fail
            return

        if self.current_mode == 0:
            self._attach_object_async(True, self._on_attach_done)  # pick
        else:
            self._attach_object_async(False, self._on_detach_done)  # place

    def _on_attach_done(self, ok):
        if not ok:
            self._finish(False)
            return
        # self._ascend()
        self._finish(True)  # success without ascend

    def _on_detach_done(self, ok):
        if not ok:
            self._finish(False)
            return
        # self._ascend()
        self._finish(True)  # success without ascend

    def _ascend(self):
        # ascend motion (not used if commented)
        p = self.motion_args
        self._call_move_line(-p["dx"], -p["dy"], -p["dz"],
                             -p["drx"], -p["dry"], -p["drz"],
                             p["vel"], p["acc"], p["ref"], p["mv_mode"],
                             self._on_ascend_done)

    def _on_ascend_done(self, ok):
        self._finish(ok)

    def _call_move_line(self, dx, dy, dz, drx, dry, drz, vel, acc, ref, mv_mode, cb):
        req = MoveLine.Request()
        req.pos = [dx, dy, dz, drx, dry, drz]  # relative pose
        req.vel = vel                          # velocity
        req.acc = acc                          # acceleration
        req.time = 0.0                         # time unused
        req.radius = 0.0                       # no blending
        req.ref = ref                          # base/tool
        req.mode = mv_mode                     # abs/rel mode
        req.blend_type = 0
        req.sync_type = 0

        future = self.move_line_cli.call_async(req)  # async call
        future.add_done_callback(lambda f: self._on_move_line_result(f, cb))  # callback

    def _on_move_line_result(self, future, cb):
        try:
            result = future.result()  # response
            cb(result.success)
        except:
            cb(False)

    def _attach_object_async(self, attach, cb):
        if not self.attach_ac.wait_for_server(timeout_sec=5.0):
            cb(False)
            return

        goal = AttachObject.Goal()
        goal.attach_object = AttachState.ATTACH.value if attach else AttachState.DETACH.value  # attach/detach
        goal.fallback_radius = 0.15
        goal.object_config = self._make_marker("grasp_frame", self.default_mesh_path)  # object marker

        future = self.attach_ac.send_goal_async(goal)  # async action
        future.add_done_callback(lambda f: self._on_goal_sent(f, cb))

    def _on_goal_sent(self, future, cb):
        try:
            handle = future.result()  # goal accepted?
            if not handle.accepted:
                cb(False)
                return

            result_future = handle.get_result_async()  # wait result
            result_future.add_done_callback(lambda f: self._on_attach_result(f, cb))
        except:
            cb(False)

    def _on_attach_result(self, future, cb):
        try:
            _ = future.result().result  # result OK
            cb(True)
        except:
            cb(False)

    def _finish(self, ok):
        if not self.result_future.done():
            self.result_future.set_result(ok)  # complete service

    def _make_marker(self, frame_id, mesh_path):
        m = Marker()                     # marker for mesh object
        m.header.frame_id = frame_id
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = mesh_path
        m.pose = Pose()
        m.pose.orientation.w = 1.0
        m.pose.position.z = 0.035
        m.scale = Vector3(x=1.0, y=1.0, z=1.0)
        m.color.g = 1.0
        m.color.a = 1.0
        return m


def main(args=None):
    rclpy.init(args=args)
    n = PickPlaceServer()  # start node
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
