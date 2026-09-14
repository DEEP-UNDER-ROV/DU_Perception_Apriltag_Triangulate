cmake_minimum_required(VERSION 3.14)
project(jir_rectification_remap_lib)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}
  src/rectification_remapper.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link OpenCV
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${OpenCV_LIBRARIES}
)

# Link ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  sensor_msgs
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export for downstream packages
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs OpenCV)

ament_package()
