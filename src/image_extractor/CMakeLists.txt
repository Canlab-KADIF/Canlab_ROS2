cmake_minimum_required(VERSION 3.5)
project(image_extractor)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 패키지 종속성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 실행 파일 추가
add_executable(extractor src/extractor.cpp)

# 라이브러리 링크
ament_target_dependencies(extractor
  rclcpp
  sensor_msgs
  rosbag2_cpp
  cv_bridge
  OpenCV
)

install(TARGETS
  extractor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

