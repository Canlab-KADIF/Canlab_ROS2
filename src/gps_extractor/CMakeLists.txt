cmake_minimum_required(VERSION 3.5)
project(gps_extractor)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 패키지 종속성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# 실행 파일 추가
add_executable(extractor src/extractor.cpp)

# 라이브러리 링크
ament_target_dependencies(extractor
  rclcpp
  tf2_msgs
  geometry_msgs
  SQLite3
  rosidl_runtime_cpp
)

# 설치 설정
install(TARGETS
  extractor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

