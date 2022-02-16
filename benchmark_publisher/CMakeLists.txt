cmake_minimum_required(VERSION 2.8.3)
project(benchmark_publisher)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -rdynamic")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    )

catkin_package()
include_directories(${catkin_INCLUDE_DIRS}) 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(benchmark_publisher
    src/benchmark_publisher_node.cpp
    )

target_link_libraries(benchmark_publisher ${catkin_LIBRARIES})
