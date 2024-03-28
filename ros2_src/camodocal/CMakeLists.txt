cmake_minimum_required(VERSION 3.8)
project(camodocal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
            filesystem program_options system) 

include_directories(${Boost_INCLUDE_DIRS} 
                    ${CERES_INCLUDE_DIRS} 

                    )

add_library(camodocal src/chessboard/Chessboard.cc
                        src/calib/CameraCalibration.cc
                        src/camera_models/Camera.cc
                        src/camera_models/CameraFactory.cc
                        src/camera_models/CostFunctionFactory.cc
                        src/camera_models/PinholeCamera.cc
                        src/camera_models/CataCamera.cc
                        src/camera_models/EquidistantCamera.cc
                        src/camera_models/ScaramuzzaCamera.cc
                        src/sparse_graph/Transform.cc
                        src/gpl/gpl.cc
                        src/gpl/EigenQuaternionParameterization.cc)

target_include_directories(camodocal PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

target_include_directories(camodocal PUBLIC ${PROJECT_SOURCE_DIR}/include/)
target_link_libraries(camodocal ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${camodocal_lib})

# ament_export_targets(camodocal HAS_LIBRARY_TARGET)

ament_target_dependencies(
  camodocal
  OpenCV
  rclcpp
  Ceres
  Boost
)

ament_export_libraries(camodocal)

install(
  DIRECTORY include/camodocal
  DESTINATION include
)

install(
  TARGETS camodocal
  EXPORT camodocal
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)

ament_package()