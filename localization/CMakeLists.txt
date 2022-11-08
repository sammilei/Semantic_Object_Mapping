cmake_minimum_required(VERSION 2.8.3)
project(artifact_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   cv_bridge
   image_geometry
   image_transport
   tf
   tf_conversions
   eigen_conversions
   darknet_ros_msgs
   vision_msgs
   visualization_msgs
   artifact_detection_rf
   apriltags2_ros
   pcl_ros
   artifact_msgs
)

find_package(Eigen3 REQUIRED)

# These Boost modules are needed for GTSAM (version 4.1+).
# This must be called before find_packaging PCL and GTSAM
# as Boost only processes the first occurence
find_package(Boost REQUIRED COMPONENTS
  date_time
  system
  timer
  chrono
  serialization
  system
  filesystem
  thread
  program_options
  date_time
  timer
  chrono
  regex
  atomic
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV_FOUND)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
  endif()
endif()

find_package(PCL 1.7 REQUIRED)

find_package(GTSAM REQUIRED)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} 
  src/camera.cpp
  src/config.cpp
  src/localizer.cpp
  src/artifact.cpp
  src/signal_processor.cpp
  # src/gas_strength_processor.cpp
  src/observation.cpp
  src/kalman.cpp
  src/batch_optimizer.cpp
  src/msl_raptor_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${realsense2_LIBRARY}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    gtsam
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${realsense2_LIBRARY}
    ${OpenCV_LIBRARIES}
) 

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #add_rostest_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.test test/test_${PROJECT_NAME}.cpp)
  #target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})

add_rostest_gtest(test_artifact
  test/test_artifact.test
  test/test_artifact.cpp)
target_link_libraries(test_artifact
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_rostest_gtest(test_batch_optimizer
  test/test_batch_optimizer.test
  test/test_batch_optimizer.cpp)
target_link_libraries(test_batch_optimizer
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_rostest_gtest(test_artifact_localization_unit_1
  test/test_artifact_localization_unit_1.test
  test/test_artifact_localization_unit_1.cpp
  test/test_artifact_localization_common.cpp)
target_link_libraries(test_artifact_localization_unit_1
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_rostest_gtest(test_artifact_localization_unit_2
  test/test_artifact_localization_unit_2.test
  test/test_artifact_localization_unit_2.cpp
  test/test_artifact_localization_common.cpp)
target_link_libraries(test_artifact_localization_unit_2
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_rostest_gtest(test_artifact_localization_pipeline
  test/test_artifact_localization_pipeline.test
  test/test_artifact_localization_pipeline.cpp
  test/test_artifact_localization_common.cpp)
target_link_libraries(test_artifact_localization_pipeline
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

endif()
