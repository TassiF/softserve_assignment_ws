cmake_minimum_required(VERSION 3.8)
project(manipulator_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 Jazzy build dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ur_description REQUIRED)
find_package(xacro REQUIRED)

add_executable(
  isaac_joint_trajectory_controller
  src/isaac_joint_trajectory_controller.cpp
)
ament_target_dependencies(
  isaac_joint_trajectory_controller
  builtin_interfaces
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

add_executable(
  random_pick_place_planner
  src/random_pick_place_planner.cpp
)
ament_target_dependencies(
  random_pick_place_planner
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  shape_msgs
  std_msgs
)

install(
  DIRECTORY config launch scripts urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  PROGRAMS
    scripts/spawn_ur30.py
  DESTINATION
    share/${PROJECT_NAME}/scripts
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS
    isaac_joint_trajectory_controller
    random_pick_place_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Register the package with ament.
ament_package()
