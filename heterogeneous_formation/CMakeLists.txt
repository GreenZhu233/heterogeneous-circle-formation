cmake_minimum_required(VERSION 3.8)
project(heterogeneous_formation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(formation_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(formation_gazebo_worlds REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(quadrotor_landing src/quadrotor_landing.cpp)
ament_target_dependencies(quadrotor_landing
  rclcpp
  rclcpp_action
  formation_interfaces
  nav_msgs
)

add_executable(formation_2_auto_control_node src/formation_2_auto_control.cpp)
ament_target_dependencies(formation_2_auto_control_node
  rclcpp
  rclcpp_action
  formation_interfaces
  nav_msgs
  rcl_interfaces
  lifecycle_msgs
)

ament_package()

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    quadrotor_landing
    formation_2_auto_control_node
  DESTINATION lib/${PROJECT_NAME}
)