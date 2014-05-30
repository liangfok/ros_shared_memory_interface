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

find_package(Boost REQUIRED COMPONENTS thread python)
find_package(PythonLibs REQUIRED)
include_directories(include ../common/include SYSTEM ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

#shared memory library
add_library(shared_memory_interface src/shared_memory_transport.cpp src/shared_memory_interface.cpp)
target_link_libraries(shared_memory_interface ${Boost_LIBRARIES} -lrt)

#watchdog (shared memory cleanup tool)
rosbuild_add_executable(sm_watchdog src/sm_watchdog.cpp)
target_link_libraries(sm_watchdog shared_memory_interface)

#echo
#rosbuild_add_executable(echo src/echo.cpp)
#target_link_libraries(echo shared_memory_interface ${PYTHON_LIBRARIES})

#benchmark
rosbuild_add_executable(benchmark src/benchmark.cpp)
target_link_libraries(benchmark shared_memory_interface ${PYTHON_LIBRARIES})
rosbuild_add_executable(benchmark_mirror src/benchmark_mirror.cpp)
target_link_libraries(benchmark_mirror shared_memory_interface ${PYTHON_LIBRARIES})