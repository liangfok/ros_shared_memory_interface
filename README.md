# Introduction #

This library provides a shared-memory-based transport layer with an API similar to ROS topic publishers and subscribres.

# Basic Test #
To run basic test:

    $ roscore
    $ rosrun shared_memory_interface shared_memory_manager
    $ rosrun shared_memory_interface_tutorials tutorial_ros_talker
    $ rosrun shared_memory_interface_tutorials tutorial_ros_listener

# Round Trip Time Benchmark #

To run a basic benchmark:

    $ roscore
    $ rosrun shared_memory_interface shared_memory_manager
    $ rosrun shared_memory_interface_tutorials tutorial_rtt_slave
    $ rosrun shared_memory_interface_tutorials tutorial_rtt_master

Here are the results on a Lenovo T430 Laptop with an Intel(R) Core(TM) i7-3520M CPU @ 2.90GHz, Ubtuntu 14.04, and 3.13.0-53-generic 64-bit kernel:

    [ INFO] [1432613206.576842765]: RTT Benchmark statistics:
     - Num samples: 10000
     - Size samples: 1
     - Average (us): 5.8537
     - Standard deviation: 4.53723
     - Min (us): 4.151
     - Max (us): 272.118
