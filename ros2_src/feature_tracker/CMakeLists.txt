cmake_minimum_required(VERSION 3.8)
project(feature_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camodocal REQUIRED)

file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(feature_tracker_node ${SRC_FILE})

ament_target_dependencies(
  feature_tracker_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  message_filters
  OpenCV
  camodocal
)

install(
  TARGETS  feature_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()