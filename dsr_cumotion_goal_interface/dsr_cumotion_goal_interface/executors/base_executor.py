import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes


class MoveItExecutorBase:
    """비동기 방식으로 MoveGroup 액션을 제어하는 기본 실행기 클래스."""

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

    # --------------------------
    # 내부 유틸리티
    # --------------------------
    def _reset_state(self):
        """기존 goal을 취소하고 초기화."""
        if self.current_goal_handle:
            try:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda _: self.node.get_logger().info("[Executor] Previous goal canceled.")
                )
            except Exception as e:
                self.node.get_logger().warn(f"[Executor] Cancel goal failed: {e}")
        self.current_goal_handle = None

    def _wait_for_server(self) -> bool:
        """MoveGroup 서버 연결 대기."""
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("[Executor] MoveGroup server not available (timeout 5s).")
            return False
        return True

    # --------------------------
    # Goal 전송 (비동기)
    # --------------------------
    def send_goal(self, request, description: str, vel_scale=None, acc_scale=None, on_complete=None):
        """
        비동기 방식으로 MoveGroup goal 전송.
        완료되면 on_complete 콜백 호출 (선택사항).
        """
        self._reset_state()
        if not self._wait_for_server():
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request

        info = f"(vel={vel_scale or 'default'}, acc={acc_scale or 'default'})"
        self.node.get_logger().info(f"[Executor] Sending goal: {description} {info}")

        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        send_future.add_done_callback(lambda f: self._on_goal_response_async(f, description, on_complete))

        return True

    def _on_goal_response_async(self, future, description: str, on_complete=None):
        """Goal 수락 여부 처리."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn(f"[Executor] Goal rejected: {description}")
                return

            self.current_goal_handle = goal_handle
            self.node.get_logger().info(f"[Executor] Goal accepted: {description}")

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self._on_result_async(f, description, on_complete))

        except Exception as e:
            self.node.get_logger().error(f"[Executor] Goal response error: {e}")

    # --------------------------
    # 피드백 & 결과 콜백
    # --------------------------
    def _on_feedback(self, feedback_msg):
        """MoveGroup 실행 중 피드백."""
        fb = feedback_msg.feedback
        state = getattr(fb, "state", None)
        if state:
            self.node.get_logger().info(f"[Feedback] {state}")

    def _on_result_async(self, future, description: str, on_complete=None):
        """최종 결과 처리 및 다음 명령 준비."""
        try:
            result = future.result().result
            code = result.error_code.val
            code_name = self._error_code_name(code)

            if code == MoveItErrorCodes.SUCCESS:
                self.node.get_logger().info(f"[Result] SUCCESS: {description} ({code_name})")
            else:
                self.node.get_logger().warn(f"[Result] FAILED: {description} ({code_name}, code={code})")

            if on_complete:
                on_complete(code == MoveItErrorCodes.SUCCESS)

        except Exception as e:
            self.node.get_logger().error(f"[Result] Exception while handling result: {e}")

        finally:
            self.current_goal_handle = None
            self.node.get_logger().info("[Executor] Ready for next goal.")

    # --------------------------
    # 에러 코드 매핑
    # --------------------------
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
