cmake_minimum_required(VERSION 3.8)
project(rov_vision)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher
  rclcpp
  sensor_msgs
  camera_info_manager
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(stereo_triangulator src/stereo_triangulator.cpp)
ament_target_dependencies(stereo_triangulator
  rclcpp
  geometry_msgs
  Eigen3
  apriltag_msgs
  sensor_msgs
  message_filters
)

target_include_directories(
  stereo_triangulator PUBLIC include)

install(TARGETS camera_info_publisher stereo_triangulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
