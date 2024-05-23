cmake_minimum_required(VERSION 2.8.3)
project(data_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs cv_bridge image_transport)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(data_generator
    src/data_generator_node.cpp
    src/data_generator.cpp
    )

target_link_libraries(data_generator ${catkin_LIBRARIES} ${OpenCV_LIBS})
