cmake_minimum_required(VERSION 3.0.2)
project(lqr_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros 
  mavlink 
  mavros_msgs
  custom_libs
)

include_directories( include ${catkin_INCLUDE_DIRS} )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs nav_msgs
)

###########
## Build ##
###########

add_executable(lqr_control src/lqr_control.cpp)
target_link_libraries(lqr_control ${catkin_LIBRARIES} )



