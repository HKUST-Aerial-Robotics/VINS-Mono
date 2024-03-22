cmake_minimum_required(VERSION 3.8)
project(ar_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(camodocal REQUIRED)


# find_package(camodocal REQUIRED)
file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(ar_demo_node ${SRC_FILE})

ament_target_dependencies(
  ar_demo_node
  rclcpp
  std_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  message_filters
  image_transport
  camodocal
)

install(
  TARGETS 
  ar_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
