cmake_minimum_required(VERSION 3.5)
project(autoware_planning_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneSequence.msg"
  "msg/Path.msg"
  "msg/PathPoint.msg"
  "msg/PathWithLaneId.msg"
  "msg/PathPointWithLaneId.msg"
  "msg/Route.msg"
  "msg/RouteSection.msg"
  "msg/Scenario.msg"
  "msg/StopFactor.msg"
  "msg/StopReason.msg"
  "msg/StopReasonArray.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# Export package dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install message files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# Package build
ament_package()
