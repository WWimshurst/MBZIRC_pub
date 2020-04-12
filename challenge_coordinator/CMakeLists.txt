cmake_minimum_required(VERSION 2.8.3)
project(challenge_coordinator)


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  rospy
  message_generation
)
add_service_files(DIRECTORY srv
        FILES
        CommandMisc.srv
        )

generate_messages(DEPENDENCIES std_msgs sensor_msgs)


catkin_package(CATKIN_DEPENDS sensor_msgs std_msgs rospy)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Install launch files
install(
        DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
