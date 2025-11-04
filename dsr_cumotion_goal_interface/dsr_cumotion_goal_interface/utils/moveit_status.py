#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveIt2 Action 상태 모니터링 노드
- goal 전송 없이 /move_action/status 토픽을 구독
- 현재 진행 중인 goal 상태(PLANNING, EXECUTING, SUCCEEDED 등)를 출력
"""

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray


class MoveItActionStatusMonitor(Node):
    def __init__(self):
        super().__init__('moveit2_status_monitor')

        # ✅ MoveIt Action status 토픽 (환경에 따라 _action 접두사 포함)
        possible_topics = ['/move_action/_action/status']
        for topic in possible_topics:
            self.status_topic = topic
            break

        self.get_logger().info(f'🚀 Subscribing to {self.status_topic}')
        self.subscription = self.create_subscription(
            GoalStatusArray,
            self.status_topic,
            self.status_callback,
            10
        )

    def status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            self.get_logger().info(' No active MoveIt goals.')
            return

        self.get_logger().info('🔹 Current MoveIt Goal Statuses:')
        for status in msg.status_list:
            goal_id = self._uuid_to_str(status.goal_info.goal_id.uuid)
            state_code = status.status
            state_text = self._status_to_text(state_code)
            self.get_logger().info(f'  - Goal {goal_id[:8]}... → {state_text}')

    @staticmethod
    def _uuid_to_str(uuid_field) -> str:
        import numpy as np

        if isinstance(uuid_field, bytes):
            return uuid_field.hex()
        elif isinstance(uuid_field, np.ndarray):
            return ''.join(f'{b:02x}' for b in uuid_field.tolist())
        elif isinstance(uuid_field, list):
            return ''.join(f'{b:02x}' for b in uuid_field)
        else:
            return str(uuid_field)

    @staticmethod
    def _status_to_text(code: int) -> str:
        mapping = {
            0: 'UNKNOWN',
            1: 'ACCEPTED',
            2: 'EXECUTING',
            3: 'CANCELING',
            4: 'SUCCEEDED',
            5: 'CANCELED',
            6: 'ABORTED',
        }
        return mapping.get(code, f'UNKNOWN({code})')


def main(args=None):
    rclpy.init(args=args)
    node = MoveItActionStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received.')
    finally:
        # shutdown 중복 호출 방지
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
