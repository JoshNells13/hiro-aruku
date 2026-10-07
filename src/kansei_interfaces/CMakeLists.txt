cmake_minimum_required(VERSION 3.5)
project(kansei_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(kansei_interfaces
  "msg/Axis.msg"
  "msg/Point3.msg"
  "msg/Status.msg"
  "msg/Unit.msg")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
