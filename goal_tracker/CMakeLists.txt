cmake_minimum_required(VERSION 3.8)
project(goal_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rover_msgs REQUIRED) # 메시지를 포함하는 rover_msgs 패키지 찾기

# 실행 파일
add_executable(goal_tracker_node src/goal_tracker_node.cpp)
ament_target_dependencies(goal_tracker_node rclcpp tf2 tf2_ros geometry_msgs rover_msgs)

# 메시지 인터페이스를 노드에 연결
# rosidl_target_interfaces(goal_tracker_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS goal_tracker_node
  DESTINATION lib/${PROJECT_NAME})

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
