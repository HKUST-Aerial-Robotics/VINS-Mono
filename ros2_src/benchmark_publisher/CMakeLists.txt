cmake_minimum_required(VERSION 3.8)
project(benchmark_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(benchmark_publisher_node ${SRC_FILE})

ament_target_dependencies(
  benchmark_publisher_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(
  TARGETS 
  benchmark_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
