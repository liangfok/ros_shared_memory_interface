/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Chien-Liang Fok
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
// #include "shared_memory_interface/shared_memory_publisher.hpp"
#include "shared_memory_interface/shared_memory_subscriber.hpp"
#include "std_msgs/Float64.h"

#define WRITE_TO_ROS_TOPIC false
#define LISTEN_TO_ROS_TOPIC false
#define USE_POLLING true

bool firstRcv1 = true;
bool firstRcv2 = true;
bool firstRcv3 = true;

static void printRcvTime(std::string callbackName, double value)
{
  char buffer[30];
  struct timeval tv;
  time_t curtime;

  gettimeofday(&tv, NULL); 
  curtime=tv.tv_sec;
          
  strftime(buffer,30,"%m-%d-%Y  %T.", localtime(&curtime));

  ROS_INFO_STREAM("Subscriber: " << callbackName << " called, time = " << buffer << tv.tv_usec << ", value = " << value);
}

void callback1(std_msgs::Float64& msg)
{
  if (firstRcv1)
  {
    printRcvTime("callback1", msg.data);
    firstRcv1 = false;
  }
}

void callback2(std_msgs::Float64& msg)
{
  if (firstRcv2)
  {
    printRcvTime("callback2", msg.data);
    firstRcv2 = false;
  }
}

void callback3(std_msgs::Float64& msg)
{
  if (firstRcv3)
  {
    printRcvTime("callback3", msg.data);
    firstRcv3 = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  // Declare three subscribers
  shared_memory_interface::Subscriber<std_msgs::Float64> sub1(LISTEN_TO_ROS_TOPIC, USE_POLLING);
  shared_memory_interface::Subscriber<std_msgs::Float64> sub2(LISTEN_TO_ROS_TOPIC, USE_POLLING);
  shared_memory_interface::Subscriber<std_msgs::Float64> sub3(LISTEN_TO_ROS_TOPIC, USE_POLLING);
  sub1.subscribe("/topic1", boost::bind(&callback1, _1));
  sub1.subscribe("/topic2", boost::bind(&callback2, _1));
  sub1.subscribe("/topic3", boost::bind(&callback3, _1));

  ros::Rate loop_rate(1000);
  while (ros::ok() && (firstRcv1 || firstRcv2 || firstRcv3))
  {
    loop_rate.sleep();
  }

  ROS_INFO("Subscriber done, press ctrl+c to exit...");
  ros::spin();
  return 0;
}
