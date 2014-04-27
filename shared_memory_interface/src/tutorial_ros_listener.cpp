/*
 * File: tutorial_ros_listener.cpp
 * Package: shared_memory_interface
 * Author: Joshua James
 * License: CC BY-SA 3.0 (attribution required)
 */

#include "ros/ros.h"
#include "shared_memory_interface/shared_memory_interface_ros.hpp"
#include "std_msgs/String.h"

using namespace shared_memory_interface;

void chatterCallback(std_msgs::String& msg)
{
  ROS_INFO("I heard: [%s]", msg.data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  SharedMemoryInterfaceROS smi("smi");
  smi.subscribeSerializedROS<std_msgs::String>("chatter", boost::bind(&chatterCallback, _1));

  ros::spin();

  return 0;
}

