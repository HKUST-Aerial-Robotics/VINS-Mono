# add library
include_directories(./)
add_library(${PROJECT_NAME} cv_bridge.cpp rgb_colors.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

if(NOT ANDROID)
# add a Boost Python library
find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

#Get the numpy include directory from its python module
if(NOT PYTHON_NUMPY_INCLUDE_DIR)
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
                    RESULT_VARIABLE PYTHON_NUMPY_PROCESS
                    OUTPUT_VARIABLE PYTHON_NUMPY_INCLUDE_DIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(PYTHON_NUMPY_PROCESS EQUAL 0)
       file(TO_CMAKE_PATH "${PYTHON_NUMPY_INCLUDE_DIR}" PYTHON_NUMPY_INCLUDE_CMAKE_PATH)
       set(PYTHON_NUMPY_INCLUDE_DIR ${PYTHON_NUMPY_INCLUDE_CMAKE_PATH} CACHE PATH "Numpy include directory")
    else(PYTHON_NUMPY_PROCESS EQUAL 0)
        message(SEND_ERROR "Could not determine the NumPy include directory, verify that NumPy was installed correctly.")
    endif(PYTHON_NUMPY_PROCESS EQUAL 0)
 endif(NOT PYTHON_NUMPY_INCLUDE_DIR)

include_directories(${PYTHON_INCLUDE_PATH} ${Boost_INCLUDE_DIRS} ${PYTHON_NUMPY_INCLUDE_DIR})

if (PYTHON_VERSION_MAJOR VERSION_EQUAL 3)
  add_definitions(-DPYTHON3)
endif()

if (OpenCV_VERSION_MAJOR VERSION_EQUAL 3)
add_library(${PROJECT_NAME}_boost module.cpp module_opencv3.cpp)
else()
add_library(${PROJECT_NAME}_boost module.cpp module_opencv2.cpp)
endif()
target_link_libraries(${PROJECT_NAME}_boost ${Boost_LIBRARIES}
                                            ${catkin_LIBRARIES}
                                            ${PYTHON_LIBRARIES}
                                            ${PROJECT_NAME}
)

set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/boost/
                      PREFIX ""
)
if(APPLE)
  set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                        SUFFIX ".so")
endif()

install(TARGETS ${PROJECT_NAME}_boost DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost/)
endif()
