# add the tests

# add boost directories for now
include_directories("../src")

catkin_add_gtest(${PROJECT_NAME}-utest test_endian.cpp utest.cpp utest2.cpp test_rgb_colors.cpp)
target_link_libraries(${PROJECT_NAME}-utest
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_nosetests(enumerants.py)
catkin_add_nosetests(conversions.py)
catkin_add_nosetests(python_bindings.py)
