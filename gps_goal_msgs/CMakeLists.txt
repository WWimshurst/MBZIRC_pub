cmake_minimum_required(VERSION 2.8.3)
project(gps_goal_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  actionlib_msgs
)


## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action FILES
   ExecuteWaypoints.action
   ExecuteWaypointsGPS.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs std_msgs actionlib_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_navigation_msgs
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
