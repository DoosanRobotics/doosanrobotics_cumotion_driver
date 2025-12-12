import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup  # [MOD] Prevent Action waitable race
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
import threading


class MoveItExecutorBase:

    # Initialize ActionClient with thread-safe goal management
    def __init__(
        self,
        node: Node,
        group_name="manipulator",
        pipeline_id="isaac_ros_cumotion",
        base_frame="base_link",
        tool_frame="grasp_frame",
    ):
        self.node = node
        self.group_name = group_name
        self.pipeline_id = pipeline_id
        self.base_frame = base_frame
        self.tool_frame = tool_frame

        self.cb_group = ReentrantCallbackGroup()  # [MOD] Dedicated Reentrant callback group for Action
        self.client = ActionClient(
            node,
            MoveGroup,
            "move_action",
            callback_group=self.cb_group,          # [MOD] Enable concurrent safe waitable execution
        )

        self.current_goal_handle = None
        self._goal_lock = threading.Lock()

    # Wait for the MoveGroup action server with retry attempts
    def _wait_for_server(self, timeout_sec=5.0, retry_count=3) -> bool:
        for attempt in range(retry_count):
            if self.client.wait_for_server(timeout_sec=timeout_sec):
                return True
            self.node.get_logger().warn(
                f"[Executor] MoveGroup server not available "
                f"(attempt {attempt + 1}/{retry_count})"
            )

        self.node.get_logger().error(
            "[Executor] MoveGroup server unavailable. "
            "Please ensure move_group node is running."
        )
        return False

    # Cancel the currently active goal if one exists
    def cancel_current_goal(self):
        with self._goal_lock:
            goal_to_cancel = self.current_goal_handle
            self.current_goal_handle = None

        if goal_to_cancel is not None:
            try:
                self.node.get_logger().info("[Executor] Canceling current goal...")
                cancel_future = goal_to_cancel.cancel_goal_async()
                cancel_future.add_done_callback(self._on_cancel_done)
            except Exception as e:
                self.node.get_logger().error(f"[Executor] Cancel error: {e}")

    # Handle the cancellation result from the action server
    def _on_cancel_done(self, future):
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.node.get_logger().info("[Executor] Goal canceled successfully.")
            else:
                self.node.get_logger().warn("[Executor] Goal cancellation failed.")
        except Exception as e:
            self.node.get_logger().error(f"[Executor] Cancel callback error: {e}")

    # Send a new goal after safely canceling any existing goal
    def send_goal(self, request, description: str,
                  vel_scale=None, acc_scale=None,
                  on_complete=None, cancel_previous=True):

        # [MOD] Enforce strict serialization: send a goal only after cancel completes
        def _send_after_cancel():
            if not self._wait_for_server():
                if on_complete:
                    on_complete(False)
                return

            try:
                goal_msg = MoveGroup.Goal()
                goal_msg.request = request

                info = f"(vel={vel_scale or 'default'}, acc={acc_scale or 'default'})"
                self.node.get_logger().info(
                    f"[Executor] Sending goal: {description} {info}"
                )

                send_future = self.client.send_goal_async(
                    goal_msg, feedback_callback=self._on_feedback
                )
                send_future.add_done_callback(
                    lambda f: self._on_goal_response_async(f, description, on_complete)
                )

            except Exception as e:
                self.node.get_logger().error(f"[Executor] Failed to send goal: {e}")
                if on_complete:
                    on_complete(False)

        with self._goal_lock:
            if self.current_goal_handle is not None:
                if cancel_previous:
                    self.node.get_logger().warn(
                        "[Executor] Canceling previous goal before sending a new one."
                    )
                    goal_to_cancel = self.current_goal_handle
                else:
                    self.node.get_logger().warn(
                        "[Executor] Previous goal is still active. Rejecting new goal."
                    )
                    return False
            else:
                goal_to_cancel = None

        if goal_to_cancel is not None:
            try:
                cancel_future = goal_to_cancel.cancel_goal_async()

                # [MOD] Trigger new goal only after cancel completion
                def _after_cancel(_):
                    self.node.get_logger().info("[Executor] Previous goal canceled.")
                    _send_after_cancel()

                cancel_future.add_done_callback(_after_cancel)  # [MOD] Cancel-to-send chaining

            except Exception as e:
                self.node.get_logger().error(f"[Executor] Cancel error: {e}")
                _send_after_cancel()
        else:
            _send_after_cancel()

        return True

    # Process goal acceptance or rejection from the action server
    def _on_goal_response_async(self, future, description: str, on_complete=None):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn(f"[Executor] Goal rejected: {description}")
                if on_complete:
                    on_complete(False)
                return

            with self._goal_lock:
                self.current_goal_handle = goal_handle

            self.node.get_logger().info(f"[Executor] Goal accepted: {description}")

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self._on_result_async(f, description, on_complete)
            )

        except Exception as e:
            self.node.get_logger().error(f"[Executor] Goal response error: {e}")
            if on_complete:
                on_complete(False)

    # Handle real-time feedback from the action server
    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        state = getattr(fb, "state", None)
        if state:
            self.node.get_logger().info(f"[Feedback] {state}")

    # Handle the final execution result from the action server
    def _on_result_async(self, future, description: str, on_complete=None):
        success = False
        try:
            result = future.result().result
            code = result.error_code.val
            code_name = self._error_code_name(code)

            if code == MoveItErrorCodes.SUCCESS:
                self.node.get_logger().info(
                    f"[Result] SUCCESS: {description} ({code_name})"
                )
                success = True
            else:
                self.node.get_logger().warn(
                    f"[Result] FAILED: {description} ({code_name}, code={code})"
                )

        except Exception as e:
            self.node.get_logger().error(
                f"[Result] Exception while handling result: {e}"
            )

        finally:
            with self._goal_lock:
                self.current_goal_handle = None

            self.node.get_logger().info("[Executor] Ready for next goal.")

            if on_complete:
                try:
                    on_complete(success)
                except Exception as e:
                    self.node.get_logger().error(
                        f"[Result] Error in on_complete callback: {e}"
                    )

    # Convert MoveIt error code to a human-readable name
    @staticmethod
    def _error_code_name(code: int) -> str:
        mapping = {
            1: "SUCCESS",
            -1: "FAILURE",
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -4: "MOTION_PLAN_INVALIDATED",
            -5: "CONTROL_FAILED",
            -6: "SENSOR_DATA_UNAVAILABLE",
            -7: "TIMED_OUT",
            -8: "PREEMPTED",
            -9: "START_STATE_IN_COLLISION",
            -10: "START_STATE_VIOLATES_CONSTRAINTS",
            -11: "GOAL_IN_COLLISION",
            -12: "GOAL_VIOLATES_CONSTRAINTS",
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
