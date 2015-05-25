# Introduction #

This library provides a shared-memory-based transport layer with an API similar to ROS topic publishers and subscribres.

# Basic Test #
To run basic test:

    $ roscore
    $ rosrun shared_memory_interface shared_memory_manager 
    $ rosrun shared_memory_interface_tutorials tutorial_ros_talker
    $ rosrun shared_memory_interface_tutorials tutorial_ros_listener