cmake_minimum_required(VERSION 3.0)
project(feature_tracker VERSION 1.0)
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(OpenCV REQUIRED)
find_package(camodocal REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

# Define your library target
add_library(feature_tracker STATIC src/feature_tracker.cpp)
target_link_libraries(feature_tracker PRIVATE ${OpenCV_LIBS} ${CERES_LIBRARIES} spdlog::spdlog_header_only camodocal::camodocal )

# Set include directories
target_include_directories(feature_tracker
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Install the library
include(GNUInstallDirs)
install(TARGETS feature_tracker
    EXPORT feature_trackerTargets
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
install(EXPORT feature_trackerTargets
    FILE feature_trackerTargets.cmake
    NAMESPACE feature_tracker::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/feature_tracker
)


# Generate and install CMake package configuration files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/feature_trackerConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/feature_trackerConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/feature_trackerConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/feature_tracker
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/feature_trackerConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/feature_trackerConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/feature_tracker
)