cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(shared_memory_interface src/shared_memory_transport.cpp src/shared_memory_interface.cpp)

rosbuild_add_executable(sm_watchdog src/sm_watchdog.cpp)
target_link_libraries(sm_watchdog shared_memory_interface)

rosbuild_add_executable(tutorial_talker src/tutorial_talker.cpp)
target_link_libraries(tutorial_talker shared_memory_interface)

rosbuild_add_executable(tutorial_listener src/tutorial_listener.cpp)
target_link_libraries(tutorial_listener shared_memory_interface)