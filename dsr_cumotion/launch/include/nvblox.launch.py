import os
from typing import List, Tuple

import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu
import isaac_manipulator_ros_python_utils.constants as constants

# camera functions from isaac_manipulator
def get_realsense_remappings(num_cameras: int) -> List[Tuple[str, str]]:
    remappings = []
    for i in range(num_cameras):
        remappings.append((f'/camera_{i}/color/image', f'/camera_{i+1}/color/image_raw'))
        remappings.append((f'/camera_{i}/color/camera_info', f'/camera_{i+1}/color/camera_info'))
        remappings.append((f'/camera_{i}/depth/camera_info',
                           f'/camera_{i+1}/aligned_depth_to_color/camera_info'))
        # If we have no robot, nvblox consumes the depth image directly.
        # Otherwise the depth image published by the robot segmenter is used.
        remappings.append((f'/camera_{i}/depth/image', f'/cumotion/camera_{i+1}/world_depth'))
    return remappings


def get_sim_remappings() -> List[Tuple[str, str]]:
    remappings = []
    remappings.append(('/camera_0/color/image', '/rgb'))
    remappings.append(('/camera_0/color/camera_info', '/camera_info'))
    # We want nvblox to consume the depth map that segments out the robot
    # remappings.append(('/camera_0/depth/image', '/depth'))

    remappings.append(('/camera_0/depth/image', '/cumotion/camera_1/world_depth'))
    remappings.append(('/camera_0/depth/camera_info', '/camera_info'))
    return remappings

def add_nvblox(args: lu.ArgumentContainer) -> List[lut.Action]:
    camera_type = str(args.camera_type).lower()
    num_cameras = int(args.num_cameras)
    workspace_bounds_name = str(args.workspace_bounds_name)
    actions = []

    # Check if the configuration is valid
    if camera_type == "realsense":
        assert num_cameras <= 2, 'Running more than 2 RealSense cameras not allowed.'
    elif camera_type == "isaac_sim":
        assert num_cameras == 1, 'Running multiple cameras in Isaac Sim not allowed.'

    # Get config files
    base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml')
    custom_base_config = lu.get_path('dsr_cumotion',
                                          'config/nvblox_custom.yaml')
    workspace_config = lu.get_path('dsr_cumotion',
                                   f'config/{workspace_bounds_name}.yaml')

    # Get remappings and specialized parameters
    if camera_type == "realsense":
        remappings = get_realsense_remappings(num_cameras)
    elif camera_type == "isaac_sim":
        remappings = get_sim_remappings()
    else:
        raise Exception(f'CameraType {camera_type} not implemented.')

    # Load the workspace config
    if not os.path.exists(workspace_config):
        raise Exception(f'Workspace with name {workspace_bounds_name} does not exist. '
                        'Launching nvblox without valid workspace is not allowed.')

    # Get all parameters with overrides.
    parameters = []
    parameters.append(base_config)
    parameters.append(custom_base_config)
    parameters.append(workspace_config)
    parameters.append({'num_cameras': num_cameras})

    nvblox_node = lut.ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters)

    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info([
            "Enabling nvblox for '",
            str(num_cameras), "' '",
            str(camera_type), "' cameras configured for the '", workspace_bounds_name,
            "' workspace."
        ]))
    return actions

def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('camera_type')
    args.add_arg('num_cameras', 1)
    args.add_arg('workspace_bounds_name', 'workbound_test')
    args.add_arg('container_name', constants.MANIPULATOR_CONTAINER_NAME)

    args.add_opaque_function(add_nvblox)
    return lut.LaunchDescription(args.get_launch_actions())