cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#ROS tutorial
rosbuild_add_executable(tutorial_ros_talker src/tutorial_ros_talker.cpp)
rosbuild_add_executable(tutorial_ros_listener src/tutorial_ros_listener.cpp)

#Benchmark
rosbuild_add_executable(tutorial_rtt_master src/tutorial_rtt_master.cpp)
target_link_libraries(tutorial_rtt_master ${Boost_LIBRARIES} -lrt)

rosbuild_add_executable(tutorial_rtt_slave src/tutorial_rtt_slave.cpp)
target_link_libraries(tutorial_rtt_slave ${Boost_LIBRARIES} -lrt)
