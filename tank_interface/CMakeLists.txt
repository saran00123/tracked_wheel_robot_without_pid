cmake_minimum_required(VERSION 3.8)
project(tank_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorRPMArray.msg"
)

# Install message files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
