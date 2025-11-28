cmake_minimum_required(VERSION 3.8)
project(dsr_cumotion_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPose.msg"
  "msg/TargetJoint.msg"
  "msg/TargetNamed.msg"
  "msg/TargetRelative.msg"
  "srv/PickPlace.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
