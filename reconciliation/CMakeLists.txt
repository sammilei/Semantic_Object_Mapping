cmake_minimum_required(VERSION 2.8.3)
project(artifact_reconciliation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs
  sensor_msgs
  cv_bridge
  artifact_msgs
  #CATKIN_DEPENDS message_runtime
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    cv_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME})

catkin_install_python(PROGRAMS scripts/get_netvlad_desc_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)

  # Define path of test images folder here.
  find_path(TEST_PATH
    NAMES "test_image_1.jpg"
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/test/")
  message("Test dir = ${TEST_PATH}")
  add_definitions(-DTEST_IMAGES_PATH="${TEST_PATH}")
  
  find_package(rostest REQUIRED COMPONENTS
    cv_bridge OpenCV
  )
  include_directories(${OpenCV_INCLUDE_DIRS})

  add_rostest_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.test test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})
endif()