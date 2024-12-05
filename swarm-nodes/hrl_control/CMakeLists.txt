cmake_minimum_required(VERSION 3.0.2)
project(hrl_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros 
  mavlink 
  mavros_msgs
  custom_libs
)

include_directories( include ${catkin_INCLUDE_DIRS} )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mavros_msgs geometry_msgs nav_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(hrl_control src/hrl_control.cpp)
target_link_libraries(hrl_control ${catkin_LIBRARIES} )