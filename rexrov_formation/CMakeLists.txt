cmake_minimum_required(VERSION 3.8)
project(rexrov_formation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(plankton REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

include_directories("/usr/include/eigen3")

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

add_executable(circle_formation_lifecycle_node src/circle_formation_lifecycle.cpp)
ament_target_dependencies(
  circle_formation_lifecycle_node
  rclcpp
  rclcpp_lifecycle
  plankton
  geometry_msgs
  nav_msgs
)

install(TARGETS
  circle_formation_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  robots
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
