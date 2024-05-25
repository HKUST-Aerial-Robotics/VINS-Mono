cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    )

find_package(OpenCV REQUIRED)
find_package(feature_tracker REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(camodocal REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)

catkin_package()

add_executable(feature_tracker_node
    src/feature_tracker_node.cpp src/ros_parameter_reader.cpp
    )

target_link_libraries(feature_tracker_node ${catkin_LIBRARIES} ${OpenCV_LIBS} feature_tracker camodocal)
