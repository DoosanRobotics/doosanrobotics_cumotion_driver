import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Generate MoveIt2 + RViz node (CuMotion integrated)
def get_moveit_group_node(context):
    model = LaunchConfiguration("model").perform(context)
    use_sim = str(LaunchConfiguration("use_sim_time").perform(context)).lower()
    use_sim_bool = use_sim in ["true", "1", "yes"]
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()
    pkg_share = get_package_share_directory("dsr_cumotion")

    # File paths
    controller_file = os.path.join(pkg_share, "config", "moveit_controllers.yaml")
    kinematics_file = os.path.join(pkg_share, "config", "kinematics.yaml")
    joint_limits_file = os.path.join(pkg_share, "config", "joint_limits.yaml")
    urdf_file = "m1013_with_vgc10.urdf.xacro" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.urdf.xacro"
    srdf_file = "m1013_with_vgc10.srdf.xacro" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.srdf.xacro"
    urdf_path = os.path.join(pkg_share, "urdf", urdf_file)
    srdf_path = os.path.join(pkg_share, "srdf", srdf_file)

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(model, package_name="dsr_cumotion")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .trajectory_execution(file_path=controller_file)
        .to_moveit_configs()
    )

    # Load CuMotion and OMPL configs
    cumotion_path = os.path.join(pkg_share, "config", "isaac_ros_cumotion_planning.yaml")
    ompl_path = os.path.join(pkg_share, "config", "ompl_planning.yaml")
    with open(cumotion_path) as f:
        cumotion_config = yaml.safe_load(f)
    with open(ompl_path) as f:
        ompl_config = yaml.safe_load(f)

    # Inject pipeline configurations
    pipelines = moveit_config.planning_pipelines.get("planning_pipelines", [])
    if isinstance(pipelines, list):
        pipelines.insert(0, "isaac_ros_cumotion")
        pipelines.insert(1, "ompl")
    moveit_config.planning_pipelines["isaac_ros_cumotion"] = cumotion_config
    moveit_config.planning_pipelines["ompl"] = ompl_config
    moveit_config.planning_pipelines["default_planning_pipeline"] = "isaac_ros_cumotion"

    if use_sim_bool:
        moveit_config.trajectory_execution["trajectory_execution"]["allowed_start_tolerance"] = 0.0

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_bool})
    moveit_dict = moveit_config.to_dict()
    moveit_dict["capabilities"] = "move_group/ExecuteTaskSolutionCapability"

    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    nodes = [move_group_node]

    # Optionally include RViz2
    gui = str(LaunchConfiguration("gui").perform(context)).lower()
    if gui in ["true", "1", "yes"]:
        rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
            parameters=[moveit_dict, {"use_sim_time": use_sim_bool}],
        )
        nodes.append(rviz_node)
    return nodes

# Include CuMotion pipeline (Isaac ROS CuMotion)
def get_cumotion_node(context):
    use_sim_bool = str(LaunchConfiguration("use_sim_time").perform(context)).lower() in ["true", "1", "yes"]
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()
    enable_cumotion = str(LaunchConfiguration("enable_cumotion").perform(context)).lower()
    enable_attach = str(LaunchConfiguration("enable_attach").perform(context)).lower()

    pkg_share = get_package_share_directory("dsr_cumotion")
    nodes = []
    urdf = "m1013_with_vgc10.urdf" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.urdf"
    xrdf = "m1013_with_vgc10.xrdf" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.xrdf"
    urdf_path = os.path.join(pkg_share, "urdf", urdf)
    xrdf_path = os.path.join(pkg_share, "xrdf", xrdf)

    if enable_cumotion in ["true", "1", "yes"]:
        cumotion_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "include", "cumotion.launch.py")),
            launch_arguments={
                "camera_type": "isaac_sim",
                "num_cameras": "1",
                "workspace_bounds_name": "workbound_test",
                "enable_object_attachment": enable_attach,
                "read_esdf_world": "False",
                "tool_frame": "grasp_frame",
                "joint_states_topic": "/joint_states",
                "use_sim_time": str(use_sim_bool),
                "urdf_file_path": urdf_path,
                "robot_file_name": xrdf_path,
                "gripper": gripper,
            }.items(),
        )
        nodes.append(cumotion_launch)

    if enable_attach in ["true", "1", "yes"]:
        static_depth_node = Node(
            package="dsr_cumotion",
            executable="camera_publisher.py",
            name="static_depth_camera_node",
            output="log",
        )
        nodes.append(static_depth_node)
    return nodes

# Include NVBlox mapping pipeline
def get_nvblox_node(context):
    use_sim_bool = str(LaunchConfiguration("use_sim_time").perform(context)).lower() in ["true", "1", "yes"]
    enable_nvblox = str(LaunchConfiguration("enable_nvblox").perform(context)).lower()
    nodes = []

    if enable_nvblox in ["true", "1", "yes"]:
        launch_dir = os.path.join(get_package_share_directory("dsr_cumotion"), "launch", "include")
        nvblox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, "/nvblox.launch.py"]),
            launch_arguments={
                "camera_type": "isaac_sim",
                "use_sim_time": str(use_sim_bool),
                "workspace_bounds_name": "workbound_test",
            }.items(),
        )
        nodes.append(nvblox_launch)
    return nodes


def set_urdf_xacro_fn(context):
    model = LaunchConfiguration("model").perform(context)
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower() in [
        "true",
        "1",
        "yes",
    ]
    urdf_file = (
        f"{model}_with_vgc10.urdf.xacro" if gripper else f"{model}_without_gripper.urdf.xacro"
    )
    xacro_path = os.path.join(
        get_package_share_directory("dsr_cumotion"), "urdf", urdf_file
    )
    return [SetLaunchConfiguration("urdf_xacro_path", xacro_path)]

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

def obstacle_manager_fn(context):
    obstacle_flag = str(LaunchConfiguration("obstacle").perform(context)).lower() in ["true","1","yes",]
    pkg_share = get_package_share_directory("dsr_cumotion")
    yaml_path = os.path.join(pkg_share, "config", "obstacles.yaml")
    nodes = []
    if obstacle_flag:
        obstacle_node = Node(
            package="dsr_cumotion",
            executable="obstacle_manager.py",
            name="obstacle_manager",
            output="screen",
            parameters=[{"config_file": yaml_path}],
        )
        nodes.append(obstacle_node)
    return nodes

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
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time"),
        DeclareLaunchArgument("gripper", default_value="true", description="GRIPPER"),
        DeclareLaunchArgument("obstacle", default_value="true", description="Obstacle using moveit planningscene"),
        DeclareLaunchArgument("enable_nvblox", default_value="true", description="Enable nvblox node"),
        DeclareLaunchArgument("enable_cumotion", default_value="true", description="Enable cumotion node"),
        DeclareLaunchArgument("enable_attach", default_value="true", description="Enable object_attach node"),
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

    set_robot_description = SetLaunchConfiguration("robot_description", robot_description_content)

    manipulator_container = Node(
        package="rclcpp_components",
        executable="component_container_mt",
        name="manipulator_container",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

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
            {"robot_description": ParameterValue(LaunchConfiguration("robot_description"), value_type=str)}
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

    motion_command = Node(
        package="dsr_cumotion_goal_interface",
        executable="move_command_node",
        name="move_command_node",
        output="screen",
        parameters = [{
            'planning_group': 'manipulator',
            'planner_pipeline': 'isaac_ros_cumotion',
            'planner_id': 'cuMotion',
            'base_frame': 'base_link',
            'tool_frame': 'grasp_frame',
            'allowed_planning_time': 5.0,
            'num_planning_attempts': 10,
            'max_vel_scale': 1.0,
            'max_acc_scale': 1.0,
        }]
    )

    object_attach_node = Node(
        package="dsr_cumotion",
        executable="object_attach_client.py",
        name="object_attach_client",
        output="screen",
    )

    cumotion = OpaqueFunction(function=get_cumotion_node)
    nvblox = OpaqueFunction(function=get_nvblox_node)
    moveit_group = OpaqueFunction(function=get_moveit_group_node)
    obstacle = OpaqueFunction(function=obstacle_manager_fn)

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
            TimerAction(period=13.0, actions=[nvblox]),
            TimerAction(period=15.0, actions=[moveit_group]),
            TimerAction(period=20.0, actions=[obstacle]),
            TimerAction(period=20.0, actions=[motion_command]),
            TimerAction(period=22.0, actions=[object_attach_node]),
        ]
    )
