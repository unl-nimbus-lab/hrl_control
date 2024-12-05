cmake_minimum_required(VERSION 3.0.2)
project(altitude_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros 
  mavlink 
  mavros_msgs
  custom_libs
  swarm_comms
)

include_directories(include ${catkin_INCLUDE_DIRS} )

# generate_messages(
# 	DEPENDENCIES
# 	std_msgs
# 	geometry_msgs
# 	)


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs nav_msgs message_runtime
)

include(MavrosMavlink)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(altitude_controller src/altitude_controller.cpp)
target_link_libraries(altitude_controller ${catkin_LIBRARIES})