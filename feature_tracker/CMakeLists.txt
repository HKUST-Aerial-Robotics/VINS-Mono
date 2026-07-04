cmake_minimum_required(VERSION 3.8)
project(feature_tracker)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_model REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    )

ament_target_dependencies(feature_tracker
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    camera_model
)

target_link_libraries(feature_tracker camera_model::camera_model ${OpenCV_LIBS} -rdynamic)

target_include_directories(feature_tracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${camera_model_INCLUDE_DIRS})

install(TARGETS feature_tracker
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_tracking_safety test/test_tracking_safety.cpp)
    target_include_directories(test_tracking_safety PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

ament_package()
