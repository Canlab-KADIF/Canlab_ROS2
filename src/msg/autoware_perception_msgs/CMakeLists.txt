cmake_minimum_required(VERSION 3.5)
project(autoware_perception_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED) # 'uuid_msgs' is replaced with 'unique_identifier_msgs' in ROS 2

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/object_recognition/DynamicObject.msg"
  "msg/object_recognition/DynamicObjectArray.msg"
  "msg/object_recognition/Shape.msg"
  "msg/object_recognition/Semantic.msg"
  "msg/object_recognition/State.msg"
  "msg/object_recognition/Feature.msg"
  "msg/object_recognition/PredictedPath.msg"
  "msg/object_recognition/DynamicObjectWithFeature.msg"
  "msg/object_recognition/DynamicObjectWithFeatureArray.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs unique_identifier_msgs
)

# Export package dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install message files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# Package build
ament_package()