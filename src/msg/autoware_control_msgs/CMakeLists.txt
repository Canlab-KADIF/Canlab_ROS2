cmake_minimum_required(VERSION 3.5)
project(autoware_control_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Define message files
set(MSG_FILES
  "msg/GateMode.msg"
  "msg/ControlCommand.msg"
  "msg/ControlCommandStamped.msg"
)

# Generate the ROS2 interface (messages, services, etc.)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
    std_msgs
)

# Install the message files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

# Package configuration
ament_package()