cmake_minimum_required(VERSION 3.8)
project(tank_drive_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tank_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add the executable
add_executable(tank_drive_node src/tank_drive_controller.cpp)
ament_target_dependencies(tank_drive_node
  rclcpp
  geometry_msgs
  tank_interface
)

# Install the executable
install(TARGETS
  tank_drive_node
  DESTINATION lib/${PROJECT_NAME})


# Add the executable
add_executable(advanced_tank_drive_node 
  src/advanced_tank_drive_controller.cpp
)
target_include_directories(advanced_tank_drive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(advanced_tank_drive_node
  rclcpp
  geometry_msgs
  tank_interface
  nav_msgs
  tf2
  tf2_ros
)

# Install the executable
install(TARGETS
  advanced_tank_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (e.g., launch and config)
install(DIRECTORY include launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
