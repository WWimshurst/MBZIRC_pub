cmake_minimum_required(VERSION 2.8.3)

project(gps_goal)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  actionlib
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
  gps_goal_msgs
)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy tf actionlib actionlib_msgs sensor_msgs geometry_msgs move_base_msgs
)


include_directories( ${catkin_INCLUDE_DIRS})


# Install launch files
install(
        DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
