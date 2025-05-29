cmake_minimum_required(VERSION 3.5)
project(c8nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Nav2Status.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# Python installation
ament_python_install_package(c8nav_py
  PACKAGE_DIR src/c8nav
)

# Install executable scripts
install(PROGRAMS
  src/c8nav/nav2_status_publisher.py
  src/c8nav/get_goal.py
  src/c8nav/pose_estimate.py
  src/c8nav/qr_code_aruco_create.py
  src/c8nav/qr_code_direct_navigator.py
  src/c8nav/save_current_pose.py
  src/c8nav/send_goal.py
  src/c8nav/test_location_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
