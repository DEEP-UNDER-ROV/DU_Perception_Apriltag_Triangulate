cmake_minimum_required(VERSION 3.14)
project(defraction_map_finder)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_definitions(EIGEN_DONT_VECTORIZE EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(sensor_msgs REQUIRED)
find_package(jir_refractive_image_geometry REQUIRED)
find_package(jir_refractive_image_geometry_msgs REQUIRED)

add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

# GSL for polynomial roots
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

# Create executable
add_executable(${PROJECT_NAME}
  src/defraction_map_finder.cpp
  src/MapFinder.cpp
  src/Camera.cc
  src/CameraFactory.cc
  src/CataCamera.cc
  src/EquidistantCamera.cc
  src/PinholeCamera.cc
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${gsl_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${gsl_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  jir_refractive_image_geometry
  jir_refractive_image_geometry_msgs
)

# Install executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
