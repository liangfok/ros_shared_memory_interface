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

rosbuild_genmsg()

#manager
rosbuild_add_executable(shared_memory_manager src/shared_memory_manager.cpp)
target_link_libraries(shared_memory_manager ${Boost_LIBRARIES} -lrt)

#remover
rosbuild_add_executable(shared_memory_remover src/shared_memory_remover.cpp)
target_link_libraries(shared_memory_remover ${Boost_LIBRARIES} -lrt)
