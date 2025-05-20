cmake_minimum_required(VERSION 3.5)
project(c8nav)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Nav2Status.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

ament_package()
