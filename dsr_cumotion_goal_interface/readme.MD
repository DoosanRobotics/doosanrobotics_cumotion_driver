ros2 topic pub /target_pose dsr_cumotion_goal_interface/TargetPose2 "{move_type: 'pose', x: 0.4, y: 0.2, z: 0.3, rx: 90, ry: 0, rz: 180}" --once
ros2 topic pub /target_pose dsr_cumotion_goal_interface/TargetPose2 "{move_type: 'pose', x: 0.4, y: 0.2, z: 0.3, qx: 0, qy: 0, qz: 0, qw: 1}" --once

ros2 topic pub /target_pose dsr_cumotion_goal_interface/TargetPose2 "{move_type: 'joint', joints: [0.0, 0.0, 0.0, 0.0, 180.0, 0.0]}" --once

ros2 topic pub /target_pose dsr_cumotion_goal_interface/TargetPose2 "{move_type: 'named', name: 'HOME'}" --once
ros2 topic pub /target_pose dsr_cumotion_goal_interface/TargetPose2 "{move_type: 'named', name: 'SET'}" --once