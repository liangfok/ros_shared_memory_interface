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

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(include ../common/include SYSTEM ${Boost_INCLUDE_DIRS})

#shared memory library
add_library(shared_memory_interface src/shared_memory_transport.cpp src/shared_memory_interface.cpp)
target_link_libraries(shared_memory_interface ${Boost_LIBRARIES})

#watchdog (shared memory cleanup tool)
rosbuild_add_executable(sm_watchdog src/sm_watchdog.cpp)
target_link_libraries(sm_watchdog shared_memory_interface)


#ROS tutorial
rosbuild_add_executable(tutorial_ros_talker src/tutorial_ros_talker.cpp)
target_link_libraries(tutorial_ros_talker shared_memory_interface)

rosbuild_add_executable(tutorial_ros_listener src/tutorial_ros_listener.cpp)
target_link_libraries(tutorial_ros_listener shared_memory_interface)


#non-ROS tutorial
add_executable(tutorial_sensor src/tutorial_sensor.cpp)
target_link_libraries(tutorial_sensor shared_memory_interface -lrt)

add_executable(tutorial_controller src/tutorial_controller.cpp)
target_link_libraries(tutorial_controller shared_memory_interface -lrt)

add_executable(tutorial_actuator src/tutorial_actuator.cpp)
target_link_libraries(tutorial_actuator shared_memory_interface -lrt)