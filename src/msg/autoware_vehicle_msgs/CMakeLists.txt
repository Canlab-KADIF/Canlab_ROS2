cmake_minimum_required(VERSION 3.5)
project(autoware_vehicle_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)

# Define message files
set(MSG_FILES
  "msg/VehicleCommand.msg"
  "msg/Shift.msg"
  "msg/ShiftStamped.msg"
  "msg/ControlMode.msg"
  "msg/TurnSignal.msg"
  "msg/Steering.msg"
  "msg/RawVehicleCommand.msg"
  "msg/RawControlCommand.msg"
  "msg/RawControlCommandStamped.msg"
)

# Generate the ROS2 interface (messages, services, etc.)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    autoware_control_msgs
)

# Install the message files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

# Package configuration
ament_package()