cmake_minimum_required(VERSION 2.8.3)
project(artifact_detection_rf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########
#add_dependencies(${PROJECT_NAME}_node)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
#target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
#catkin_add_gtest(test_${PROJECT_NAME}_fixture test/test_${PROJECT_NAME}_fixture.cpp)
#target_link_libraries(test_${PROJECT_NAME}_fixture ${PROJECT_NAME} ${catkin_LIBRARIES})

