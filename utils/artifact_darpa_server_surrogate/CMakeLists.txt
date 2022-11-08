cmake_minimum_required(VERSION 2.8.3)
project(darpa_server_surrogate)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

# Testing
if (CATKIN_ENABLE_TESTING)
  #catkin_add_nosetests(tests)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_scoring.test)
endif()
