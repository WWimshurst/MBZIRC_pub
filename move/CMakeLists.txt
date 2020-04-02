cmake_minimum_required(VERSION 2.8.3)
project(move)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  move_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

include_directories( ${catkin_INCLUDE_DIRS})


# Install launch files
install(
        DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
