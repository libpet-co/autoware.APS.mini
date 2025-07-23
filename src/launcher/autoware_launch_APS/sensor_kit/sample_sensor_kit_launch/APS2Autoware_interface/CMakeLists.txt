cmake_minimum_required(VERSION 3.8)
project(motor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(message_filters REQUIRED)


include_directories(
  include
)

add_library(motor_controller
  src/motor_controller.cpp
)

ament_target_dependencies(motor_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  serial
)

add_executable(motor2auto src/motor2auto.cpp)
ament_target_dependencies(
  motor2auto 
  autoware_vehicle_msgs
  rclcpp 
  std_msgs 
  geometry_msgs 
  serial
  nav_msgs
  tf2)

add_executable(auto2motor src/auto2motor.cpp)
ament_target_dependencies(auto2motor 
  rclcpp
  autoware_control_msgs
  geometry_msgs
  std_msgs)
target_link_libraries(motor2auto
  motor_controller
)

add_executable(test_cmd src/test_cmd.cpp)
ament_target_dependencies(test_cmd
  autoware_control_msgs
  rclcpp 
  std_msgs 
  tf2)

add_executable(cmd_monitor src/cmd_monitor.cpp)
  ament_target_dependencies(cmd_monitor
    autoware_control_msgs
    geometry_msgs
    message_filters
    sensor_msgs
    rclcpp 
    std_msgs 
    tf2)


install(TARGETS
  motor2auto
  auto2motor
  motor_controller
  test_cmd
  cmd_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
