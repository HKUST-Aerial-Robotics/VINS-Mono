cmake_minimum_required(VERSION 3.0)
project(camodocal VERSION 1.0)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
    filesystem
    program_options
)
# include_directories(${CERES_INCLUDE_DIRS})
# Add your source files
set(SOURCES
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CataCamera.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/chessboard/Chessboard.cc
    src/gpl/gpl.cc
    src/sparse_graph/Transform.cc
    # Add more source files if needed
)

# Define your library target
add_library(camodocal SHARED ${SOURCES})
target_link_libraries(camodocal PRIVATE ${OpenCV_LIBS} ${CERES_LIBRARIES})

# Set include directories
target_include_directories(camodocal
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Set up your executable
add_executable(intrinsic_calibration
    src/intrinsic_calib.cc 
)

# Link your library to the executable
target_link_libraries(intrinsic_calibration PRIVATE ${OpenCV_LIBS} ${Boost_LIBRARIES} camodocal )

# Install the library
include(GNUInstallDirs)
install(TARGETS camodocal
    EXPORT camodocalTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Generate and install CMake package configuration files
install(EXPORT camodocalTargets
    FILE camodocalTargets.cmake
    NAMESPACE camodocal::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/camodocal
)

# Install the executable
install(TARGETS intrinsic_calibration
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Generate and install CMake package configuration files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/camodocalConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/camodocalConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/camodocalConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/camodocal
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/camodocalConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/camodocalConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/camodocal
)