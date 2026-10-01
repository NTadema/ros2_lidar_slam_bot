cmake_minimum_required(VERSION 3.8)

project(esp32_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------
# Dependencies
# -------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# -------------------------------------------------
# Shared library
# -------------------------------------------------

add_library(${PROJECT_NAME}_lib
  src/bridge_node.cpp
  src/serial_protocol.cpp
  src/uart_linux.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC
  cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)


# -------------------------------------------------
# Main executable
# -------------------------------------------------

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)


# -------------------------------------------------
# Install
# -------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


# -------------------------------------------------
# Export
# -------------------------------------------------

ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)


# -------------------------------------------------
# Package
# -------------------------------------------------

ament_package()
