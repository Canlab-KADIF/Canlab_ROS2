cmake_minimum_required(VERSION 3.5)
project(canlab_listen)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
)
        
add_executable(${PROJECT_NAME}_node src/canlab_listen.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs)

install(TARGETS
        ${PROJECT_NAME}_node 
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})
        
ament_package()
