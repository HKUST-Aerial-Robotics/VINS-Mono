cmake_minimum_required(VERSION 2.8)
project(cv_bridge_1_11)

find_package(catkin REQUIRED COMPONENTS rosconsole sensor_msgs)

if(NOT ANDROID)
  find_package(PythonLibs)
  if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
    find_package(Boost REQUIRED python)
  else()
    find_package(Boost REQUIRED python3)
  endif()
else()
find_package(Boost REQUIRED)
endif()
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosconsole sensor_msgs
  DEPENDS OpenCV
)

catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

if(NOT ANDROID)
add_subdirectory(python)
endif()
add_subdirectory(src)
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
