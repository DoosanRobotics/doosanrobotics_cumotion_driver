from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dsr_cumotion_goal_interface',
            executable='move_command_node',
            name='move_command_node',
            output='screen',
            parameters=[{
                # 기본 MoveIt 그룹 설정
                'planning_group': 'manipulator',
                'planner_pipeline': 'isaac_ros_cumotion',
                'planner_id': 'cuMotion',

                # 로봇 프레임 설정
                'base_frame': 'base_link',
                'tool_frame': 'grasp_frame',

                # 플래닝 옵션
                'allowed_planning_time': 5.0,
                'num_planning_attempts': 10,

                # 스케일 기본값 (토픽에서 지정하지 않으면 여기 사용됨)
                'max_vel_scale': 1.0,
                'max_acc_scale': 1.0,
            }],
        ),
    ])
