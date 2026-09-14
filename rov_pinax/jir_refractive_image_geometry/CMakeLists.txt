cmake_minimum_required(VERSION 3.14)
project(jir_refractive_image_geometry)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(jir_refractive_image_geometry_msgs REQUIRED)

# GSL via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

# Header-only INTERFACE library
add_library(${PROJECT_NAME} INTERFACE)

target_compile_definitions(${PROJECT_NAME} INTERFACE
  EIGEN_DONT_VECTORIZE
  EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${gsl_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  Eigen3::Eigen
  ${gsl_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} INTERFACE
  sensor_msgs
  image_geometry
  jir_refractive_image_geometry_msgs
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export target
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  sensor_msgs
  image_geometry
  jir_refractive_image_geometry_msgs
)

ament_package()
