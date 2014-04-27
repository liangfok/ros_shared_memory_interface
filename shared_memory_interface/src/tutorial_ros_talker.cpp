/*
 * File: tutorial_ros_talker.cpp
 * Package: shared_memory_interface
 * Author: Joshua James
 * License: CC BY-SA 3.0 (attribution required)
 */

#include "ros/ros.h"
#include "shared_memory_interface/shared_memory_interface_ros.hpp"
#include "std_msgs/String.h"

using namespace shared_memory_interface;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  SharedMemoryInterfaceROS smi("smi");
  smi.advertiseSerializedROS<std_msgs::String>("chatter");

  ros::Rate loop_rate(10);
  int count = 0;
  while(ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    smi.publishSerializedROS("chatter", msg);
    loop_rate.sleep();
        ++count;
  }

  return 0;
}
