cmake_minimum_required(VERSION 3.8)
project(wheel_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 build system
find_package(ament_cmake REQUIRED)

# ROS 2 dependencies used by the node
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Pure C++ differential-drive odometry library
add_library(differential_drive_odometry
  src/differential_drive_odometry.cpp
)

target_include_directories(differential_drive_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Wheel odometry ROS 2 node
add_executable(wheel_odometry_node
  src/wheel_odometry_node.cpp
)

target_link_libraries(wheel_odometry_node
  differential_drive_odometry
)

ament_target_dependencies(wheel_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

# Install the library
install(
  TARGETS differential_drive_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the node
install(
  TARGETS wheel_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the public header
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
