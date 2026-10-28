cmake_minimum_required(VERSION 3.5)
project(growing_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(moveit_template src/moveit_template.cpp)
ament_target_dependencies(moveit_template
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  geometry_msgs
  std_msgs
  tf2
  moveit_msgs
)

add_executable(nav_template src/nav_template.cpp)
ament_target_dependencies(nav_template
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
)

add_executable(cmd_vel_stamper src/cmd_vel_stamper.cpp)
ament_target_dependencies(cmd_vel_stamper
  geometry_msgs
  rclcpp
)

add_executable(pick_state_machine src/pick_state_machine.cpp)
ament_target_dependencies(pick_state_machine
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  nav2_msgs
  moveit_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS moveit_template nav_template cmd_vel_stamper pick_state_machine
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch meshes models urdf worlds rviz config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
