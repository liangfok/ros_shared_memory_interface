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
#include "shared_memory_interface/shared_memory_publisher.hpp"
#include "shared_memory_interface/shared_memory_subscriber.hpp"
// #include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <iostream>

#define WRITE_TO_ROS_TOPIC false
#define LISTEN_TO_ROS_TOPIC false
#define USE_POLLING true

shared_memory_interface::Publisher<std_msgs::Float64> pub1(WRITE_TO_ROS_TOPIC);
shared_memory_interface::Publisher<std_msgs::Float64> pub2(WRITE_TO_ROS_TOPIC);
shared_memory_interface::Publisher<std_msgs::Float64> pub3(WRITE_TO_ROS_TOPIC);

// void callback1(std_msgs::Float64& msg)
// {
  // pub.publish(msg);
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  pub1.advertise("/robot1");
  pub2.advertise("/robot2");
  pub3.advertise("/robot3");

  // Wait for user input
  system("pause");

  std_msgs::Float64 msg1, msg2, msg3;

  // shared_memory_interface::Subscriber<std_msgs::Float64MultiArray> sub1(LISTEN_TO_ROS_TOPIC, USE_POLLING);
  // sub.subscribe("/controller1", boost::bind(&callback1, _1));

  bool firstTime = true;
  double loopCount = 0;

  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
    msg1.data = msg2.data = msg3.data = loopCount++;

    if (firstTime)
    {
      char buffer[30];
      struct timeval tv;
      time_t curtime;

      gettimeofday(&tv, NULL); 
      curtime=tv.tv_sec;
              
      strftime(buffer,30,"%m-%d-%Y  %T.", localtime(&curtime));

      ROS_INFO_STREAM("Robot publishing all three messages for first time at time " << buffer << tv.tv_usec);
      firstTime = false;
    }

    if (!pub1.publish(msg1))
    {
         ROS_ERROR("Robot: Failed to publish message on /robot1. Aborting.");
         break;
    }

    if (!pub1.publish(msg2))
    {
         ROS_ERROR("Robot: Failed to publish message on /robot2. Aborting.");
         break;
    }

    if (!pub1.publish(msg3))
    {
         ROS_ERROR("Robot: Failed to publish message on /robot3. Aborting.");
         break;
    }

    loop_rate.sleep();
  }
  return 0;
}
