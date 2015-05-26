# Introduction #

This library provides a shared-memory-based transport layer with an API similar to ROS topic publishers and subscribres.

# Installation in a Test workspace #

Create a test ROS Catkin workspace:

    $ source /opt/ros/indigo/setup.bash
    $ mkdir -p ~/rsmc_workspace/src
    $ cd ~/rsmc_workspace/src
    $ catkin_init_workspace
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash

Edit your ~/.bashrc and add the following line:

    source $HOME/rsmc_workspace/devel/setup.bash

Add this repository to the test workspace:

    $ cd ~/rsmc_workspace/src
    $ git clone [this repo]
    $ cd ros_shared_memory_interface
    $ git checkout feature/catkin
    $ cd ~/rsmsc_workspace
    $ rm -rf build devel
    $ catkin_make

# Basic Test #
To run a basic test:

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

Results on a Lenovo T430 Laptop with an Intel(R) Core(TM) i7-3520M CPU @ 2.90GHz, Ubtuntu 14.04, and 3.13.0-53-generic 64-bit kernel:

    [ INFO] [1432613206.576842765]: RTT Benchmark statistics:
     - Num samples: 10000
     - Size samples: 1
     - Average (us): 5.8537
     - Standard deviation: 4.53723
     - Min (us): 4.151
     - Max (us): 272.118

Results from a custom PC with an Intel(R) Core(TM) i7-5820K CPU @ 3.30GHz, Ubuntu 14.04, and 3.16.0-38-generic 64-bit kernel:

    [ INFO] [1432614391.493655951]: RTT Benchmark statistics:
     - Num samples: 10000
     - Size samples: 1
     - Average (us): 7.62734
     - Standard deviation: 4.42252
     - Min (us): 3.701
     - Max (us): 78.99