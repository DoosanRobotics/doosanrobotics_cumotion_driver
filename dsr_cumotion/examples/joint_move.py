#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Send MoveGroup Action Goal with 6 Joint Constraints
--------------------------------------------------
- Sends joint goal for all 6 joints to MoveIt MoveGroup Action
- Works in ROS 2 (Humble / Jazzy)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class JointMoveClient(Node):
    def __init__(self):
        super().__init__('joint_move_client')

        # MoveGroup ActionClient 생성
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # 6개 조인트 목표 (라디안 단위)
        joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
        joint_names = [f'joint_{i+1}' for i in range(6)]

        # Goal 메시지 생성
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'  # SRDF 상의 planning group 이름

        # 각 조인트별 제약 조건 추가
        constraints = Constraints()
        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        # 액션 서버 연결 대기
        self.get_logger().info('Waiting for /move_action server...')
        self._action_client.wait_for_server()

        # 목표 전송
        self.get_logger().info('Sending joint goal to MoveIt...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveIt')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'MoveIt Result: {result.error_code.val}')
        self.get_logger().info('Motion complete ✅')


def main():
    rclpy.init()
    node = JointMoveClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
