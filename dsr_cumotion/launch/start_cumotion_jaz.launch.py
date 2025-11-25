import os
import yaml
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from dsr_bringup2.controller_config import adjust_dsr_controller_yaml, parse_joints_from_urdf
from dsr_bringup2.utils import read_update_rate


def read_params(pkg_name, params_dir, params_file_name):
    params_file = os.path.join(
        get_package_share_directory(pkg_name), params_dir, params_file_name)
    return yaml.safe_load(open(params_file, 'r'))

def launch_args_from_params(pkg_name, params_dir, params_file_name,  prefix: str = None):
    launch_args = []
    launch_configs = {}
    params = read_params(pkg_name, params_dir, params_file_name)

    for param, value in params['/**']['ros__parameters'].items():
        if value is not None:
            arg_name = param if prefix is None else f'{prefix}.{param}'
            launch_args.append(DeclareLaunchArgument(name=arg_name, default_value=str(value)))
            launch_configs[param] = LaunchConfiguration(arg_name)

    return launch_args, launch_configs

# Generate robot_description and select controller YAML based on the URDF model.
def generate_robot_description_action(context, *args, **kwargs):
    dynamic_yaml = LaunchConfiguration('dynamic_yaml').perform(context).lower() == 'true'
    model = LaunchConfiguration('model').perform(context)
    color = LaunchConfiguration('color').perform(context)
    gripper = "none"

    # Parse URDF to extract active and passive joints
    urdf_xml, active_joints, passive_joints = parse_joints_from_urdf(model, color, gripper)
    print(f"[DEBUG] model={model}, gripper={gripper}")
    print(f"[DEBUG] active_joints={active_joints}")
    print(f"[DEBUG] passive_joints={passive_joints}")

    # Decide controller YAML
    if dynamic_yaml:
        original_yaml = os.path.join(
            get_package_share_directory("dsr_controller2"),
            "config",
            "dsr_controller2.yaml"
        )
        adjusted_yaml = adjust_dsr_controller_yaml(original_yaml, active_joints, passive_joints)
        print(f"[INFO] Using dynamically generated controller.yaml: {adjusted_yaml}")
    else:
        static_yaml = os.path.join(
            get_package_share_directory("dsr_controller2"),
            "config",
            f"dsr_controller2_{model}.yaml"
        )
        if os.path.exists(static_yaml):
            adjusted_yaml = static_yaml
            print(f"[INFO] Using static controller.yaml: {adjusted_yaml}")
        else:
            adjusted_yaml = os.path.join(
                get_package_share_directory("dsr_controller2"),
                "config",
                "dsr_controller2.yaml"
            )
            print(f"[WARN] Model-specific YAML not found. Using default: {adjusted_yaml}")

    return [
        SetLaunchConfiguration('robot_description', urdf_xml),
        SetLaunchConfiguration('controller_yaml', adjusted_yaml),
    ]

def rviz_and_move_group_fn(context):
    model_value = LaunchConfiguration('model').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    pkg_share = get_package_share_directory("dsr_cumotion")
    use_sim_bool = str(LaunchConfiguration("use_sim_time").perform(context)).lower() in ["true", "1", "yes"]

    package_name = f"dsr_moveit_config_{model_value}"
    package_path = FindPackageShare(package_name).perform(context)
    print("MoveIt Config Package:", package_name)
    print("Package Path:", package_path)
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()
    controller_file = os.path.join(pkg_share, "config", "moveit_controllers.yaml")
    model = LaunchConfiguration("model").perform(context)

    urdf_file = "m1013_with_vgc10.urdf.xacro" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.urdf.xacro"
    srdf_file = "m1013_with_vgc10.srdf.xacro" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.srdf.xacro"
    urdf_path = os.path.join(pkg_share, "urdf", urdf_file)
    srdf_path = os.path.join(pkg_share, "srdf", srdf_file)

    moveit_config = (
        MoveItConfigsBuilder(model, package_name="dsr_cumotion")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=controller_file)
        .planning_pipelines(pipelines=["ompl", "chomp"],      # List of planning pipelines to load (each loaded from config/<name>_planning.yaml)
                            default_planning_pipeline="ompl", # Name of the default planning pipeline (used if none is explicitly selected)
                            load_all= False                   # If pipelines is None: True loads all from config/default packages; False loads only from config package
                            )
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
        moveit_config.trajectory_execution["trajectory_execution"]["allowed_start_tolerance"] = 1.0

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_bool})
    moveit_dict = moveit_config.to_dict()
    common_params = [
        moveit_dict,  # robot_description & robot_description_semantic from MoveitConfigbuilder
        {"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)},
    ]

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=LaunchConfiguration('name'),
        output="screen",
        parameters=common_params,
    )

    rviz_base = os.path.join(pkg_share, "config")
    rviz_full_config = os.path.join(rviz_base, "moveit_test.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=common_params,
    )
    return [run_move_group_node, rviz_node]

# sets up the parameters for the controller manager node, if 'gripper' argument is setted, it additionally loads the 'gripper_controller.yaml' file
def control_node_fn(context):
    params = [{"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)}, LaunchConfiguration('controller_yaml')]

    if LaunchConfiguration('gripper').perform(context) == 'robotiq_2f85':
        pkg_share = get_package_share_directory("dsr_controller2")
        gripper_yaml = os.path.join(pkg_share, "config", "gripper_controller.yaml")
        params.append(gripper_yaml)
        print(f"[INFO] Including gripper YAML in controller_manager: {gripper_yaml}")

    node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=params,
        output="both",
    )
    return [node]

def gripper_spawner_fn(context):
    if LaunchConfiguration('gripper').perform(context) != 'robotiq_2f85':
        return []

    return [Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "-c", "controller_manager",
        ],
        output="screen",
    )]

# Include CuMotion pipeline (Isaac ROS CuMotion)
def get_cumotion_node(context):
    enable_cumotion = str(LaunchConfiguration("enable_cumotion").perform(context)).lower()
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()

    if enable_cumotion not in ["true", "1", "yes"]:
        return []

    pkg_share = get_package_share_directory("dsr_cumotion")

    # File paths
    urdf = "m1013_with_vgc10.urdf" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.urdf"
    xrdf = "m1013_with_vgc10.xrdf" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.xrdf"

    urdf_path = os.path.join(pkg_share, "urdf", urdf)
    xrdf_path = os.path.join(pkg_share, "xrdf", xrdf)

    launch_args, launch_configs = launch_args_from_params(
        'dsr_cumotion', 'config', 'isaac_ros_cumotion_params.yaml', 'cumotion_planner'
    )

    # Override robot/xrdf path
    launch_configs["robot"] = xrdf_path
    launch_configs["urdf_path"] = urdf_path

    # Env
    env_variables = dict(os.environ)

    if str(launch_configs['enable_cuda_mps']) in ["true", "1", "yes"]:
        env_variables.update({
            "CUDA_MPS_ACTIVE_THREAD_PERCENTAGE": launch_configs["cuda_mps_active_thread_percentage"],
            "CUDA_MPS_PIPE_DIRECTORY": launch_configs["cuda_mps_pipe_directory"],
            "CUDA_MPS_CLIENT_PRIORITY": launch_configs["cuda_mps_client_priority"]
        })

    # Static Planning Scene Server
    static_planning_scene_server = Node(
        package='isaac_ros_cumotion',
        executable='static_planning_scene',
        name='static_planning_scene_server',
        output='screen',
        parameters=[{
            'moveit_collision_objects_scene_file': LaunchConfiguration('cumotion_planner.moveit_collision_objects_scene_file')

        }],
        emulate_tty=True,
    )

    # CuMotion Main Node
    cumotion_planner_node = Node(
        name='cumotion_planner',
        package='isaac_ros_cumotion',
        executable='cumotion_goal_set_planner_node',
        output='screen',
        parameters=[launch_configs],
        env=env_variables
    )
    return launch_args + [static_planning_scene_server, cumotion_planner_node]

def get_object_attach_node(context):
    enable_attach = str(LaunchConfiguration("enable_attach").perform(context)).lower()
    gripper = str(LaunchConfiguration("gripper").perform(context)).lower()

    if enable_attach not in ["true", "1", "yes"]:
        return []

    pkg_share = get_package_share_directory("dsr_cumotion")

    # File paths
    urdf = "m1013_with_vgc10.urdf" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.urdf"
    xrdf = "m1013_with_vgc10.xrdf" if gripper in ["true", "1", "yes"] else "m1013_without_gripper.xrdf"

    urdf_path = os.path.join(pkg_share, "urdf", urdf)
    xrdf_path = os.path.join(pkg_share, "xrdf", xrdf)

    launch_args, launch_configs = launch_args_from_params(
        'dsr_cumotion','config', 'object_attachment_params.yaml', 'object_attachment')

    # Override robot/xrdf path
    launch_configs["robot"] = xrdf_path
    launch_configs["urdf_path"] = urdf_path

    attach_object_server_node = Node(
        package='isaac_ros_cumotion_object_attachment',
        namespace='',
        executable='attach_object_server_node',
        name='object_attachment',
        parameters=[launch_configs],
        output='screen',
    )

    static_depth_node = Node(
        package="dsr_cumotion",
        executable="camera_publisher.py",
        name="static_depth_camera_node",
        output="log",
    )

    return launch_args + [attach_object_server_node, static_depth_node]

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='', description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='m1013', description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value='false', description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='false', description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('dynamic_yaml', default_value='false', description='Use dynamic controller.yaml'),
        DeclareLaunchArgument("gripper", default_value="true", description="GRIPPER"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time"),
        DeclareLaunchArgument("enable_cumotion", default_value="true", description="Enable cumotion node"),
        DeclareLaunchArgument("enable_attach", default_value="true", description="Enable cumotion node"),
    ]

    # Build robot_description and select controller YAML
    robot_description_action = OpaqueFunction(function=generate_robot_description_action)
    update_rate = read_update_rate() # get update_rate from yaml

    # Run set_config
    set_config_node = Node(
        package="dsr_bringup2",
        executable="set_config",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": LaunchConfiguration('name'),
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('host'),
            "port": LaunchConfiguration('port'),
            "mode": LaunchConfiguration('mode'),
            "model": LaunchConfiguration('model'),
            "gripper": "none",
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
            "update_rate": update_rate,
        }],
        output="screen",
    )

    # Run emulator
    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": LaunchConfiguration('name'),
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('host'),
            "port": LaunchConfiguration('port'),
            "mode": LaunchConfiguration('mode'),
            "model": LaunchConfiguration('model'),
            "gripper":"none",
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
        }],
        output="screen",
    )

    # Run robot_state_publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
            'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str)
        }],
    )

    # Run ros2_control_node(controller_manager)
    control_node = OpaqueFunction(function=control_node_fn)

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    # Spawn dsr_controller2
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    # Spawn dsr_moveit_controller
    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('name'),
        arguments=["dsr_moveit_controller", "-c", "controller_manager"],
    )

    pick_place_server = Node(
        package="dsr_cumotion",
        executable="pick_and_place_server.py",
        name="pick_and_place_server",
        output="screen",
    )

    # MoveGroup + (optional) RViz
    rviz_and_move_group = OpaqueFunction(function=rviz_and_move_group_fn)
    cumotion = OpaqueFunction(function=get_cumotion_node)
    attach = OpaqueFunction(function=get_object_attach_node)

    # A) After set_config exits, start controller manager and then (after a short delay) spawn joint_state_broadcaster.
    delay_control_node_after_set_config = RegisterEventHandler(
        OnProcessExit(
            target_action=set_config_node,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] set_config finished. Starting ros2_control_node..."),
                control_node,
                TimerAction(period=1.0, actions=[
                    LogInfo(msg=">> [STEP 1B] Spawning joint_state_broadcaster..."),
                    joint_state_broadcaster_spawner
                ]),
            ],
        )
    )

    # B) Once joint_state_broadcaster is active, spawn dsr_controller2 (arm controller).
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 2 COMPLETED] joint_state_broadcaster active. Starting dsr_controller2..."),
                robot_controller_spawner
            ],
        )
    )

    # C) After dsr_controller2 becomes active, (conditionally) spawn the gripper position controller.
    delay_gripper_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3A] dsr_controller2 active. (cond) starting gripper_position_controller..."),
                OpaqueFunction(function=gripper_spawner_fn),
            ],
        )
    )

    # D) After dsr_controller2 becomes active, spawn dsr_moveit_controller (MoveIt-compatible trajectory controller).
    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3 COMPLETED] dsr_controller2 active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        )
    )

    # E) After dsr_moveit_controller is active, start MoveGroup (and RViz if gui=true).
    delay_rviz_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4 COMPLETED] dsr_moveit_controller active. Launching MoveGroup (+ RViz if gui=true)..."),
                rviz_and_move_group
            ],
        )
    )

    delay_cumotion_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 5 COMPLETED] cumotion active. Launching cumotion node..."),
                cumotion
            ],
        )
    )
    
    delay_attach_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 5 COMPLETED] cumotion active. Launching cumotion node..."),
                attach
            ],
        )
    )

    delay_server_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 5 COMPLETED] cumotion active. Launching cumotion node..."),
                pick_place_server
            ],
        )
    )
    nodes = [
        LogInfo(msg=">> [START] Launching Doosan Robot Bringup with MoveIt2..."),
        robot_description_action,
        set_config_node,
        run_emulator_node,
        robot_state_pub_node,
        delay_control_node_after_set_config,
        delay_robot_controller_after_joint_state,
        delay_gripper_after_robot_controller,
        delay_dsr_moveit_controller_after_robot_controller,
        delay_rviz_after_moveit_controller,
        delay_cumotion_after_moveit_controller,
        delay_attach_after_moveit_controller,
        delay_server_after_moveit_controller
    ]

    return LaunchDescription(ARGUMENTS + nodes)
