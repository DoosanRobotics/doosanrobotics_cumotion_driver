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
                'planning_group': 'manipulator',
                'planner_pipeline': 'isaac_ros_cumotion',
                'planner_id': 'cuMotion',

                'base_frame': 'base_link',
                'tool_frame': 'grasp_frame',

                'allowed_planning_time': 5.0,
                'num_planning_attempts': 10,

                'max_vel_scale': 1.0,
                'max_acc_scale': 1.0,
            }],
        ),
    ])
