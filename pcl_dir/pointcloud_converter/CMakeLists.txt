cmake_minimum_required(VERSION 3.8)
project(pointcloud_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_converter_node src/pointcloud_converter_node.cpp)
ament_target_dependencies(pointcloud_converter_node rclcpp sensor_msgs geometry_msgs pcl_conversions rover_msgs)
target_link_libraries(pointcloud_converter_node ${PCL_LIBRARIES})

# # 메시지 인터페이스를 노드에 연결
# rosidl_target_interfaces(pointcloud_converter_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 메시지 의존성 export
# ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  pointcloud_converter_node
  DESTINATION lib/${PROJECT_NAME}
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
