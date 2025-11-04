import os
import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration, TimerAction, IncludeLaunchDescription,)

from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_moveit_group_node(context):
    model = LaunchConfiguration("model").perform(context)   # model argumnet, default:=m1013
    use_sim = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_bool = use_sim.lower() in ["true", "1"]          # use sim time argument
    pkg_share = get_package_share_directory('dsr_cumotion')
    gripper = LaunchConfiguration("gripper").perform(context)

    controller_file_name = 'moveit_controllers.yaml'
    kinematics_file_name = 'kinematics.yaml'
    joint_limits_file_name = 'joint_limits.yaml'

    if gripper == 'true':
        urdf_file_name = 'm1013.urdf.xacro'
        srdf_file_name = 'm1013.srdf.xacro'
    else:
        urdf_file_name = 'm1013_without_gripper.urdf.xacro'
        srdf_file_name = 'm1013_without_gripper.srdf.xacro'

    # file path
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name,)
    srdf_path = os.path.join(pkg_share, 'srdf', srdf_file_name,)

    kinematics_path = os.path.join(pkg_share, 'config',kinematics_file_name,)
    joint_limits = os.path.join(pkg_share, 'config', joint_limits_file_name,)
    moveit_controllers = os.path.join(pkg_share, 'config', controller_file_name,)

    moveit_config = (
        MoveItConfigsBuilder(model, package_name='dsr_cumotion') # do not apply robot description
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits)
        .trajectory_execution(file_path=moveit_controllers)
        .to_moveit_configs()
    )
    
    # Add cuMotion to list of planning piplelines
    cumotion_config_file_path = os.path.join(pkg_share,'config','isaac_ros_cumotion_planning.yaml',)
    with open(cumotion_config_file_path) as cumotion_config_file:
        cumotion_config = yaml.safe_load(cumotion_config_file)

    ompl_config_file_path = os.path.join(pkg_share,'config','ompl_planning.yaml',)
    with open(ompl_config_file_path) as ompl_config_file:
        ompl_config = yaml.safe_load(ompl_config_file)

    moveit_config.planning_pipelines['planning_pipelines'].insert(0, 'isaac_ros_cumotion') # set cuMotion as default planning pipeline
    moveit_config.planning_pipelines['planning_pipelines'].insert(1, 'ompl')
    moveit_config.planning_pipelines['isaac_ros_cumotion'] = cumotion_config
    moveit_config.planning_pipelines['ompl'] = ompl_config
    moveit_config.planning_pipelines['default_planning_pipeline'] = 'isaac_ros_cumotion'

    if use_sim_bool:
        moveit_config.trajectory_execution['trajectory_execution']['allowed_start_tolerance'] = 0.0

    moveit_config.moveit_cpp.update({'use_sim_time': use_sim_bool})

    move_it_dict = moveit_config.to_dict()
    move_it_dict['capabilities'] = 'move_group/ExecuteTaskSolutionCapability'

    # movegroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='log',
        parameters=[move_it_dict],
        arguments=['--ros-args', '--log-level', 'info'],
    )
    nodes = [move_group_node]

    # RViz
    gui = LaunchConfiguration("gui").perform(context)
    if gui == "true":
        rviz_config = os.path.join(pkg_share, 'config', 'moveit.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config],
            parameters=[move_it_dict, {'use_sim_time': use_sim_bool},],
        )
        nodes.append(rviz_node)
    else:
        print("[Launch] rviz disabled (gui:=false)")

    return nodes

def get_cumotion_node(context):
    pkg_share = get_package_share_directory('dsr_cumotion')
    pkg_share2 = get_package_share_directory('dsr_cumotion')
    cumotion_launch_path = os.path.join(pkg_share, 'launch', 'include', 'cumotion.launch.py')
    gripper = LaunchConfiguration("gripper").perform(context)

    if gripper == "true":
        urdf_file_path = os.path.join(pkg_share, 'urdf', 'm1013_gripper_attach.urdf')
        xrdf_file_path = os.path.join(pkg_share2, 'xrdf', 'm1013_gripper_attach.xrdf')

    if gripper == "false":
        urdf_file_path = os.path.join(pkg_share, 'urdf', 'm1013.urdf')
        xrdf_file_path = os.path.join(pkg_share2, 'xrdf', 'm1013.xrdf')

    cumotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cumotion_launch_path),
        launch_arguments={
            'camera_type': 'isaac_sim',                  # Use Isaac Sim camera
            'num_cameras': '1',                          # Number of cameras
            'workspace_bounds_name': 'workbound_test',   # Workspace YAML file name
            'enable_object_attachment': 'True',          # Enable object attachment feature
            'read_esdf_world': 'False',                  # Enable ESDF world (set to False if nvblox is not used)
            'tool_frame': 'grasp_frame',                 # End-effector link name
            'joint_states_topic': '/joint_states',       # Remap for joint_states topic
            'use_sim_time': 'False',                      # Use simulation time from Isaac Sim
            'urdf_file_path' : urdf_file_path,
            'robot_file_name': xrdf_file_path,
        }.items(),
    )
    return [cumotion_launch]

def get_nvblox_node(context):
    launch_files_include_dir = os.path.join(
        get_package_share_directory('dsr_cumotion'), 'launch', 'include'
    )
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_files_include_dir, '/nvblox.launch.py']),
        launch_arguments={
            'camera_type': 'isaac_sim',
            'use_sim_time': 'True',
            'workspace_bounds_name': 'workbound_test',
        }.items(),
    )

    camera_arg = LaunchConfiguration("camera").perform(context).lower()
    enable_camera = camera_arg in ["true", "1", "yes"]

    nodes = [nvblox_launch]
    if enable_camera:
        static_depth_node = Node(
            package='dsr_cumotion',
            executable='camera_publisher.py',
            name='static_depth_camera_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
        nodes.append(static_depth_node)
        print("[Launch] Static depth camera node enabled.")
    else:
        print("[Launch] Camera disabled (camera:=false)")
    return nodes


# xacro path function
def set_urdf_xacro_fn(context):
    model = LaunchConfiguration("model").perform(context)
    gripper = LaunchConfiguration("gripper").perform(context).lower() in ["true", "1", "yes"]
    urdf_file = f"{model}.urdf.xacro" if gripper else f"{model}_without_gripper.urdf.xacro"
    xacro_path = os.path.join(get_package_share_directory("dsr_cumotion"), "urdf", urdf_file)
    return [SetLaunchConfiguration("urdf_xacro_path", xacro_path)]

# control node setting function using robot_description
# control node setting function using robot_description
def control_node_fn(context):
    name = LaunchConfiguration("name")
    robot_description_param = {
        "robot_description": ParameterValue(
            LaunchConfiguration("robot_description"), value_type=str
        )
    }
    pkg_share = get_package_share_directory('dsr_cumotion')
    pkg_share_dsr = get_package_share_directory("dsr_controller2")
    controller_yaml_path = os.path.join(pkg_share_dsr, "config", "dsr_controller2.yaml")
    gripper_yaml = os.path.join(pkg_share, "config", "robotiq_controller.yaml")

    params = [robot_description_param, controller_yaml_path, gripper_yaml]

    node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=name,
        parameters=params,
        output="both",
    )
    return [node]

# gripper controller spawner
def gripper_spawner_fn(context):
    node = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["gripper_position_controller", "-c", "controller_manager"],
        output="screen",
    )
    return [node]

# CollisionObject
def obstacle_manager_fn(context):
    obstacle_flag = LaunchConfiguration("obstacle").perform(context).lower() in ["true", "1", "yes"]
    pkg_share = get_package_share_directory("dsr_cumotion")
    yaml_path =  os.path.join(pkg_share, "config", "obstacles.yaml")
    if obstacle_flag:
        node = Node(
            package="dsr_cumotion",
            executable="obstacle_manager.py",
            name="obstacle_manager",
            output="screen",
            parameters=[{"config_file": yaml_path}],
        )
        print("[Launch] Obstacle Manager node created successfully.")
        return [node]
    else:
        print("[Launch] Obstacle Manager disabled (obstacle:=false)")
        return []

# move_to_pose node
def move_to_pose_fn(context):
    pkg_share = get_package_share_directory("dsr_cumotion")
    node = Node(
        package="dsr_cumotion",
        executable="move_to_pose.py",
        name="move_to_pose_node",
        output="screen",
    )
    return [node]

def generate_launch_description():
    args = [
        DeclareLaunchArgument("name", default_value="", description="Namespace"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="Robot IP"),
        DeclareLaunchArgument("port", default_value="12345", description="Robot port"),
        DeclareLaunchArgument("mode", default_value="virtual", description="Mode"),
        DeclareLaunchArgument("model", default_value="m1013", description="Robot model"),
        DeclareLaunchArgument("color", default_value="white", description="Robot color"),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("gz", default_value="false", description="Use Gazebo"),
        DeclareLaunchArgument("rt_host", default_value="192.168.137.100", description="RT IP"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use sim time"),
        DeclareLaunchArgument("gripper", default_value="true", description="GRIPPER"),
        DeclareLaunchArgument("obstacle", default_value="true", description="Obstacle using moveit planningscene"),
        DeclareLaunchArgument("camera", default_value="true", description="Static depth camera topic"),
    ]

    set_urdf_xacro = OpaqueFunction(function=set_urdf_xacro_fn)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("urdf_xacro_path"),
            " name:=",
            LaunchConfiguration("name"),
            " host:=",
            LaunchConfiguration("host"),
            " rt_host:=",
            LaunchConfiguration("rt_host"),
            " port:=",
            LaunchConfiguration("port"),
            " mode:=",
            LaunchConfiguration("mode"),
            " model:=",
            LaunchConfiguration("model"),
            " color:=",
            LaunchConfiguration("color"),
        ]
    )

    set_robot_description = SetLaunchConfiguration(
        "robot_description", robot_description_content
    )

    manipulator_container = Node(
        package='rclcpp_components',
        executable='component_container_mt',
        name='manipulator_container',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Doosan Emulator
    run_emulator = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration("name"),
        output="screen",
        parameters=[
            {
                "name": LaunchConfiguration("name"),
                "rate": 100,
                "standby": 5000,
                "command": True,
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "mode": LaunchConfiguration("mode"),
                "model": LaunchConfiguration("model"),
                "mobile": "none",
                "rt_host": LaunchConfiguration("rt_host"),
            }
        ],
    )

    control_node = OpaqueFunction(function=control_node_fn)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        parameters=[
            {
                "robot_description": ParameterValue(
                    LaunchConfiguration("robot_description"), value_type=str
                )
            }
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
        output="screen",
    )

    dsr_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=["dsr_controller2", "-c", "controller_manager"],
        output="screen",
    )

    dsr_moveit_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=["dsr_moveit_controller", "-c", "controller_manager"],
        output="screen",
    )

    # cuMotion + MoveIt2
    cumotion = OpaqueFunction(function=get_cumotion_node)
    nvblox = OpaqueFunction(function=get_nvblox_node)
    moveit_group = OpaqueFunction(function=get_moveit_group_node)
    obstacle = OpaqueFunction(function=obstacle_manager_fn)
    move_pose = OpaqueFunction(function=move_to_pose_fn)

    # Use timer action for convenience of node execution order
    # TODO: Need to modify it later with eventhandler
    return LaunchDescription(
        args
        + [
            manipulator_container,
            set_urdf_xacro,
            set_robot_description,
            run_emulator,
            TimerAction(period=3.0, actions=[control_node]),
            robot_state_publisher,
            TimerAction(period=5.0, actions=[joint_state_broadcaster]),
            TimerAction(period=7.0, actions=[dsr_controller]),
            TimerAction(period=9.0, actions=[dsr_moveit_controller]),
            TimerAction(period=11.0, actions=[cumotion]),
            # TimerAction(period=13.0, actions=[nvblox]),
            TimerAction(period=15.0, actions=[moveit_group]),
            TimerAction(period=20.0, actions=[obstacle]),
            TimerAction(period=20.0, actions=[move_pose]),
        ]
    )