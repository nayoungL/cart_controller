cmake_minimum_required(VERSION 3.0.2)
project(point_tracking)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
 
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES point_tracking
  CATKIN_DEPENDS geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs ackermann_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
src/point_tracking_node.cpp
src/point_tracking.cpp
src/pid.cpp

include/point_tracking.h
include/pid.h)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

