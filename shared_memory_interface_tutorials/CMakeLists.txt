cmake_minimum_required(VERSION 2.8.3)
project(shared_memory_interface_tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	shared_memory_interface
	roscpp
	roslib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_shared_memory_interface
  CATKIN_DEPENDS shared_memory_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

# message("** catkin_INCLUDE_DIRS: " ${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(ros_shared_memory_interface
#   src/${PROJECT_NAME}/ros_shared_memory_interface.cpp
# )

## Declare a cpp executable
# add_executable(ros_shared_memory_interface_node src/ros_shared_memory_interface_node.cpp)
add_executable(tutorial_ros_talker src/tutorial_ros_talker.cpp)

target_link_libraries(tutorial_ros_talker
	${catkin_LIBRARIES}
	${Boost_LIBRARIES} -lrt
)

add_executable(tutorial_ros_listener src/tutorial_ros_listener.cpp)

target_link_libraries(tutorial_ros_listener
	${catkin_LIBRARIES}
	${Boost_LIBRARIES} -lrt
)

#Benchmark
add_executable(tutorial_rtt_master src/tutorial_rtt_master.cpp)
target_link_libraries(tutorial_rtt_master
	${catkin_LIBRARIES}
	${Boost_LIBRARIES} -lrt
)

add_executable(tutorial_rtt_slave src/tutorial_rtt_slave.cpp)
target_link_libraries(tutorial_rtt_slave
	${catkin_LIBRARIES}
	${Boost_LIBRARIES} -lrt
)

#Initial latency test - Single Publisher Single Subscriber
add_executable(tutorial_init_latency_test_publisher src/tutorial_init_latency_test_publisher.cpp)
target_link_libraries(tutorial_init_latency_test_publisher
	${catkin_LIBRARIES}
	${Boost_LIBRARIES} -lrt
)

add_executable(tutorial_init_latency_test_subscriber src/tutorial_init_latency_test_subscriber.cpp)
target_link_libraries(tutorial_init_latency_test_subscriber
	${catkin_LIBRARIES} 
	${Boost_LIBRARIES} -lrt
)

#Initial latency test - Dual Publisher Dual Subscriber
add_executable(tutorial_init_latency_test_controller src/tutorial_init_latency_test_controller.cpp)
target_link_libraries(tutorial_init_latency_test_controller
	${catkin_LIBRARIES}
	${Boost_LIBRARIES} -lrt
)

add_executable(tutorial_init_latency_test_robot src/tutorial_init_latency_test_robot.cpp)
target_link_libraries(tutorial_init_latency_test_robot
	${catkin_LIBRARIES}
	${Boost_LIBRARIES} -lrt
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ros_shared_memory_interface_node ros_shared_memory_interface_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ros_shared_memory_interface_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ros_shared_memory_interface ros_shared_memory_interface_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_shared_memory_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)