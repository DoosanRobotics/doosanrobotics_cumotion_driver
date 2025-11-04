cmake_minimum_required(VERSION 3.8)
project(dsr_cumotion)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(isaac_ros_common REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPose.msg"
  "msg/TargetPose2.msg"
  DEPENDENCIES std_msgs
)

# Python executables
install(PROGRAMS
  launch/include/move_to_pose.py
  launch/include/camera_publisher.py
  launch/include/obstacle_manager.py

  DESTINATION lib/${PROJECT_NAME}
)

# Shared resources (launch/config/urdf/srdf/meshes)
install(DIRECTORY
  launch
  config
  urdf
  srdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Isaac ROS version info (if available)
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package export
ament_package()
