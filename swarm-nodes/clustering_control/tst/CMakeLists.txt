cmake_minimum_required(VERSION 3.16)
project(tst)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
include_directories(include ${catkin_INCLUDE_DIRS} )
	

include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
)
# # For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(googletest)

# Now simply link against gtest or gtest_main as needed. Eg
# add_executable(clustering_control ../src/clustering_control.cpp)
# target_link_libraries(clustering_control gtest_main)
# add_test(NAME example_test COMMAND clustering_control)

# if(CATKIN_ENABLE_TESTING)
#     catkin_add_gtest(clustering_control_test clustering_control_test.cpp)
# endif(CATKIN_ENABLE_TESTING)
