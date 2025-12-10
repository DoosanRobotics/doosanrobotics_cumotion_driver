import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
import threading
# Use a small delay to avoid race condition in rclcpp_action
# This is a workaround for "Taking data from action server but no ready event" bug
import time

class MoveItExecutorBase:

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

        self.client = ActionClient(node, MoveGroup, "move_action")
        self.current_goal_handle = None
        self._goal_lock = threading.Lock()

    def _wait_for_server(self, timeout_sec=5.0, retry_count=3) -> bool:
        """Wait for MoveGroup action server with retry."""
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

    def cancel_current_goal(self):
        """Cancel the currently active goal."""
        with self._goal_lock:
            if self.current_goal_handle is not None:
                self.node.get_logger().info("[Executor] Canceling current goal...")
                try:
                    cancel_future = self.current_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self._on_cancel_done)
                except Exception as e:
                    self.node.get_logger().error(f"[Executor] Cancel error: {e}")
                    self.current_goal_handle = None

    def _on_cancel_done(self, future):
        """Handle goal cancellation response."""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.node.get_logger().info("[Executor] Goal canceled successfully.")
            else:
                self.node.get_logger().warn("[Executor] Goal cancellation failed.")
        except Exception as e:
            self.node.get_logger().error(f"[Executor] Cancel callback error: {e}")
        finally:
            with self._goal_lock:
                self.current_goal_handle = None

    def send_goal(self, request, description: str, vel_scale=None, acc_scale=None, 
                  on_complete=None, cancel_previous=True):
        """
        Send MoveGroup goal asynchronously with improved error handling.
        
        Args:
            cancel_previous: If True, cancel previous goal before sending new one
        """
        with self._goal_lock:
            # Cancel previous goal if requested
            if self.current_goal_handle is not None:
                if cancel_previous:
                    self.node.get_logger().warn(
                        "[Executor] Canceling previous goal to send new one."
                    )
                    # Unlock before calling cancel to avoid deadlock
                    goal_to_cancel = self.current_goal_handle
                    self.current_goal_handle = None
                else:
                    self.node.get_logger().warn(
                        "[Executor] Previous goal still active. Rejecting new goal."
                    )
                    return False
        
        # Cancel outside the lock if needed
        if 'goal_to_cancel' in locals() and goal_to_cancel is not None:
            try:
                cancel_future = goal_to_cancel.cancel_goal_async()
                cancel_future.add_done_callback(self._on_cancel_done)
            except Exception as e:
                self.node.get_logger().error(f"[Executor] Cancel error: {e}")

        if not self._wait_for_server():
            if on_complete:
                try:
                    on_complete(False)
                except Exception as e:
                    self.node.get_logger().error(f"[Executor] Error in on_complete: {e}")
            return False

        try:
            goal_msg = MoveGroup.Goal()
            goal_msg.request = request

            info = f"(vel={vel_scale or 'default'}, acc={acc_scale or 'default'})"
            self.node.get_logger().info(
                f"[Executor] Sending goal: {description} {info}"
            )

            send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
            send_future.add_done_callback(lambda f: self._on_goal_response_async(f, description, on_complete))
            return True

        except Exception as e:
            self.node.get_logger().error(f"[Executor] Failed to send goal: {e}")
            if on_complete:
                try:
                    on_complete(False)
                except Exception as e:
                    self.node.get_logger().error(f"[Executor] Error in on_complete: {e}")
            return False

    def _on_goal_response_async(self, future, description: str, on_complete=None):
        """Handle goal acceptance with error handling."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn(
                    f"[Executor] Goal rejected: {description}"
                )
                with self._goal_lock:
                    self.current_goal_handle = None
                if on_complete:
                    on_complete(False)
                return

            with self._goal_lock:
                self.current_goal_handle = goal_handle
            
            self.node.get_logger().info(f"[Executor] Goal accepted: {description}")
            
            # Start a timer to check goal status periodically
            self._monitor_goal_status(goal_handle, description, on_complete)

        except Exception as e:
            self.node.get_logger().error(
                f"[Executor] Goal response error: {e}"
            )
            with self._goal_lock:
                self.current_goal_handle = None
            if on_complete:
                on_complete(False)

    def _monitor_goal_status(self, goal_handle, description: str, on_complete=None):
        """Monitor goal status using timer-based polling to avoid race condition."""
        timer_ref = {'timer': None}
        
        def check_status():
            try:
                status = goal_handle.status
                
                # STATUS_SUCCEEDED = 4, STATUS_ABORTED = 6, STATUS_CANCELED = 5
                if status == 4:  # SUCCEEDED
                    self.node.get_logger().info(
                        f"[Result] SUCCESS: {description}"
                    )
                    if timer_ref['timer']:
                        timer_ref['timer'].cancel()
                        timer_ref['timer'].destroy()
                    with self._goal_lock:
                        self.current_goal_handle = None
                    self.node.get_logger().info("[Executor] Ready for next goal.")
                    if on_complete:
                        try:
                            on_complete(True)
                        except Exception as e:
                            self.node.get_logger().error(
                                f"[Result] Error in on_complete callback: {e}"
                            )
                    
                elif status in [5, 6]:  # CANCELED or ABORTED
                    self.node.get_logger().warn(
                        f"[Result] FAILED: {description} (status={status})"
                    )
                    if timer_ref['timer']:
                        timer_ref['timer'].cancel()
                        timer_ref['timer'].destroy()
                    with self._goal_lock:
                        self.current_goal_handle = None
                    self.node.get_logger().info("[Executor] Ready for next goal.")
                    if on_complete:
                        try:
                            on_complete(False)
                        except Exception as e:
                            self.node.get_logger().error(
                                f"[Result] Error in on_complete callback: {e}"
                            )
                
                # STATUS_EXECUTING = 2, STATUS_ACCEPTED = 1
                # Still running, timer will check again
                
            except Exception as e:
                self.node.get_logger().error(
                    f"[Result] Error monitoring goal status: {e}"
                )
                if timer_ref['timer']:
                    timer_ref['timer'].cancel()
                    timer_ref['timer'].destroy()
                with self._goal_lock:
                    self.current_goal_handle = None
                if on_complete:
                    try:
                        on_complete(False)
                    except Exception as e2:
                        self.node.get_logger().error(
                            f"[Result] Error in on_complete callback: {e2}"
                        )
        
        # Create repeating timer to check status every 100ms
        timer_ref['timer'] = self.node.create_timer(0.1, check_status)

    def _on_feedback(self, feedback_msg):
        """Handle MoveGroup feedback."""
        fb = feedback_msg.feedback
        state = getattr(fb, "state", None)
        if state:
            self.node.get_logger().info(f"[Feedback] {state}")

    def _on_result_async(self, future, description: str, on_complete=None):
        """Handle final result with comprehensive error handling."""
        success = False
        try:
            # Safely get result with timeout to avoid hanging
            result_response = future.result()
            
            # Check if result_response is valid
            if result_response is None:
                self.node.get_logger().error(
                    f"[Result] No result received for: {description}"
                )
            elif not hasattr(result_response, 'result'):
                self.node.get_logger().error(
                    f"[Result] Invalid result response for: {description}"
                )
            else:
                result = result_response.result
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
