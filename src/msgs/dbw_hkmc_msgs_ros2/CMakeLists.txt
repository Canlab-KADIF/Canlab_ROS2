cmake_minimum_required(VERSION 3.8)
project(dbw_hkmc_msgs_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SAMPLE.msg"
  "msg/STATUS1.msg"
  "msg/STATUS2.msg"
  "msg/STATUS3.msg"
  "msg/STATUS4.msg"
  "msg/STRRATIO.msg"
  "msg/TxGearCmd.msg"
  "msg/TxLatCmd.msg"
  "msg/TxLongCmd.msg"
  "msg/TxTurnCmd.msg"
  
  DEPENDENCIES geometry_msgs std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
