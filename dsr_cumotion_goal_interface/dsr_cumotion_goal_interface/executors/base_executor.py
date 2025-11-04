import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, MoveItErrorCodes
from action_msgs.msg import GoalStatusArray


class MoveItExecutorBase:
    """Base class for structured MoveGroup action communication with live status monitoring"""

    def __init__(
        self, node: Node, group_name="manipulator", pipeline_id="isaac_ros_cumotion",
        base_frame="base_link", tool_frame="grasp_frame"
    ):
        self.node = node
        self.group_name = group_name
        self.pipeline_id = pipeline_id
        self.base_frame = base_frame
        self.tool_frame = tool_frame

        self.client = ActionClient(node, MoveGroup, "move_action")
        self.current_goal_handle = None
        self._last_feedback_state = None
        self._last_status = None

        # subscribe to MoveGroup action status topic
        self.status_sub = node.create_subscription(
            GoalStatusArray,
            '/move_action/_action/status',
            self._on_status_topic,
            10
        )

    def _reset_state(self):
        if self.current_goal_handle:
            self.node.get_logger().info("[EXECUTOR] Cancelling previous goal...")
            try:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda _: self.node.get_logger().info("[EXECUTOR] Previous goal canceled.")
                )
            except Exception as e:
                self.node.get_logger().warn(f"[EXECUTOR] Cancel goal failed: {e}")

        self.current_goal_handle = None
        self._last_feedback_state = None
        self._last_status = None

    def _wait_for_server(self):
        self.node.get_logger().info("[EXECUTOR] Waiting for MoveGroup action server...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("[EXECUTOR] MoveGroup action server not available (timeout 5s).")
            return False
        self.node.get_logger().info("[EXECUTOR] MoveGroup action server is ready.")
        return True

    def _send_goal(self, request: MotionPlanRequest, description: str, vel_scale=None, acc_scale=None):
        """Send MotionPlanRequest to MoveGroup action server"""
        self._reset_state()
        if not self._wait_for_server():
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        scale_info = f"(vel={vel_scale or 'default'}, acc={acc_scale or 'default'})"
        self.node.get_logger().info(f"[EXECUTOR] Sending goal: {description} {scale_info}")

        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        send_future.add_done_callback(lambda fut: self._on_goal_response(fut, description))
        return True

    def _on_goal_response(self, future, description: str):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error(f"[STATUS] Goal rejected by MoveGroup ({description})")
                self._update_status("REJECTED")
                return

            self.current_goal_handle = goal_handle
            self._update_status("ACCEPTED")
            self.node.get_logger().info(f"[STATUS] Goal accepted ({description}), waiting for result...")

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda res_fut: self._on_result(res_fut, description))

        except Exception as e:
            self.node.get_logger().error(f"[EXECUTOR] Goal response error: {e}")
            self._update_status("ERROR")

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        state = getattr(fb, "state", None)
        if not state:
            return

        if state != self._last_feedback_state:
            self._last_feedback_state = state
            self.node.get_logger().info(f"[FEEDBACK] State changed: {state}")

            if state == "PLANNING":
                self._update_status("EXECUTING (Planning)")
            elif state == "EXECUTING":
                self._update_status("EXECUTING (Motion)")
            elif state == "IDLE":
                self._update_status("COMPLETED")
                self.node.get_logger().info("[FEEDBACK] Motion execution completed.")

    def _on_result(self, future, description: str):
        try:
            result = future.result().result
            code = result.error_code.val
            code_name = self._error_code_name(code)

            if code == MoveItErrorCodes.SUCCESS:
                self._update_status("SUCCEEDED")
                self.node.get_logger().info(f"[RESULT] SUCCESS ({description}) [{code_name}]")
            else:
                self._update_status("ABORTED")
                self.node.get_logger().warn(f"[RESULT] FAILED ({description}) [{code_name}] (code={code})")

        except Exception as e:
            self.node.get_logger().error(f"[RESULT] Error handling result: {e}")
            self._update_status("ERROR")

        finally:
            self.current_goal_handle = None
            self.node.get_logger().info("[EXECUTOR] Ready for next command.")

    def _update_status(self, new_status: str):
        """Log only when status changes (internal executor state)"""
        if new_status != self._last_status:
            self.node.get_logger().info(f"[STATUS_NEW] {new_status}")
            self._last_status = new_status

    def _on_status_topic(self, msg: GoalStatusArray):
        """Callback for MoveGroup /move_action/_action/status topic"""
        if not msg.status_list:
            return

        latest = msg.status_list[-1]
        status_code = latest.status
        goal_id = ''.join(f'{b:02x}' for b in latest.goal_info.goal_id.uuid[:4])
        status_name = self._status_code_name(status_code)

        self.node.get_logger().info(f"[STATUS_TOPIC] Goal={goal_id} Status={status_code} ({status_name})")

    @staticmethod
    def _status_code_name(code: int) -> str:
        mapping = {
            0: "UNKNOWN",
            1: "ACCEPTED",
            2: "EXECUTING",
            3: "CANCELING",
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED",
        }
        return mapping.get(code, "INVALID")

    @staticmethod
    def _error_code_name(code: int) -> str:
        mapping = {
            1: "SUCCESS",
            -1: "FAILURE",
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -4: "MOTION_PLAN_INVALIDATED",
            -5: "CONTROL_FAILED",
            -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -7: "TIMED_OUT",
            -8: "PREEMPTED",
            -9: "START_STATE_IN_COLLISION",
            -10: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -11: "GOAL_IN_COLLISION",
            -12: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -13: "GOAL_CONSTRAINTS_VIOLATED",
            -14: "INVALID_GROUP_NAME",
            -15: "INVALID_GOAL_CONSTRAINTS",
            -16: "INVALID_ROBOT_STATE",
            -17: "INVALID_LINK_NAME",
            -18: "INVALID_OBJECT_NAME",
            -19: "FRAME_TRANSFORM_FAILURE",
            -20: "COLLISION_CHECKING_UNAVAILABLE",
            -21: "ROBOT_STATE_STALE",
            -22: "SENSOR_INFO_STALE",
            -23: "NO_IK_SOLUTION",
        }
        return mapping.get(code, "UNKNOWN")
