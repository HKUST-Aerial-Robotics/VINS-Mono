cmake_minimum_required(VERSION 3.8)
project(benchmark_publisher)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -DEIGEN_DONT_PARALLELIZE")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

add_executable(benchmark_publisher
    src/benchmark_publisher_node.cpp
    )

ament_target_dependencies(benchmark_publisher
    rclcpp
    nav_msgs
    geometry_msgs
)

target_include_directories(benchmark_publisher PRIVATE ${EIGEN3_INCLUDE_DIR})

install(TARGETS benchmark_publisher
    DESTINATION lib/${PROJECT_NAME})

ament_package()
