cmake_minimum_required(VERSION 3.8)
project(vins_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(camodocal REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(${Boost_INCLUDE_DIRS} 
                    ${CERES_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIR}
              )

add_executable(vins_estimator_node
  src/estimator_node.cpp
  src/parameters.cpp
  src/estimator.cpp
  src/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
  src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
)

ament_target_dependencies(
  vins_estimator_node
  rclcpp
  nav_msgs
  Ceres
  tf2
  tf2_ros
  OpenCV
  # camodocal
  visualization_msgs
  std_msgs
  cv_bridge
  geometry_msgs
)

target_link_libraries(vins_estimator_node ${OpenCV_LIBS} ${CERES_LIBRARIES})

install(
  TARGETS vins_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY   launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
