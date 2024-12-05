cmake_minimum_required(VERSION 3.0.2)
project(hrl_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
  DEPENDS system_lib
)

include(MavrosMavlink)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(hrl_planner src/hrl_planner.cpp)
target_link_libraries(hrl_planner ${catkin_LIBRARIES})

add_executable(state_conditioning src/state_conditioning.cpp)
target_link_libraries(state_conditioning ${catkin_LIBRARIES})