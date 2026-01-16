cmake_minimum_required(VERSION 2.8.3)
project(robot_base_navigation)
find_package(catkin REQUIRED COMPONENTS move_base roscpp std_msgs roscpp costmap_2d dynamic_reconfigure rospy)




include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
)

add_library(simple_layer
  src/simple_layer.cpp
  include/robot_base_navigation/simple_layer.h
)
add_dependencies(simple_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_layer
  ${catkin_LIBRARIES}
)