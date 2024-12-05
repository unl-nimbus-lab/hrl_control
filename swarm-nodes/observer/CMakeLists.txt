cmake_minimum_required(VERSION 3.0.2)
project(observer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavlink
  mavros_msgs
  custom_libs
)
	
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	)

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
add_executable(observer src/observer.cpp)
target_link_libraries(observer ${catkin_LIBRARIES})