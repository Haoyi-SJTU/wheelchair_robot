cmake_minimum_required(VERSION 2.8.3)
project(robot_base_odometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  N_Robot_Topic
  N_Robot
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}${catkin_INCLUDE_DIRS} "/usr/include/eigen3/")

add_dependencies(${catkin_EXPORTED_TARGETS})

add_executable(robot_base_odometry_node src/robot_base_odometry.cpp src/robot_base_odometry.h)

target_link_libraries(robot_base_odometry_node ${catkin_LIBRARIES})


add_executable(test_odom
  src/test_odom.cpp
)
add_dependencies(test_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_odom
  ${catkin_LIBRARIES}
)

add_executable(new_robot_base_odometry
  src/new_robot_base_odometry.cpp
)
add_dependencies(new_robot_base_odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(new_robot_base_odometry
  ${catkin_LIBRARIES}
)