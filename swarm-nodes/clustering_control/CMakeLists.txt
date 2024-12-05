cmake_minimum_required(VERSION 3.0.2)
project(clustering_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
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
add_executable(clustering_control src/clustering_control.cpp)
target_link_libraries(clustering_control ${catkin_LIBRARIES})

# add_subdirectory(src)
# Testing stuff
enable_testing()
add_subdirectory (tst)


if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(parse_test tst/parse_test.cpp)
  target_link_libraries(parse_test ${catkin_LIBRARIES})

  catkin_add_gtest(flight_mode_test tst/flight_mode_test.cpp)
  target_link_libraries(flight_mode_test ${catkin_LIBRARIES})
endif()
