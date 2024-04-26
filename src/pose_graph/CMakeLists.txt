cmake_minimum_required(VERSION 3.8)
project(pose_graph)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(camodocal REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(${Boost_INCLUDE_DIRS} 
                    ${CERES_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIR}
              )

add_executable(pose_graph_node
              src/pose_graph_node.cpp
              src/pose_graph.cpp
              src/keyframe.cpp
              src/utility/CameraPoseVisualization.cpp
              src/ThirdParty/DBoW/BowVector.cpp
              src/ThirdParty/DBoW/FBrief.cpp
              src/ThirdParty/DBoW/FeatureVector.cpp
              src/ThirdParty/DBoW/QueryResults.cpp
              src/ThirdParty/DBoW/ScoringObject.cpp
              src/ThirdParty/DUtils/Random.cpp
              src/ThirdParty/DUtils/Timestamp.cpp
              src/ThirdParty/DVision/BRIEF.cpp
              src/ThirdParty/VocabularyBinary.cpp
              src/parameters_.cpp
)

ament_target_dependencies(
  pose_graph_node
  rclcpp
  nav_msgs
  Ceres
  OpenCV
  camodocal
  visualization_msgs
  std_msgs
  cv_bridge
  geometry_msgs
)

target_link_libraries(pose_graph_node ${OpenCV_LIBS} ${CERES_LIBRARIES})

install(
  TARGETS pose_graph_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
