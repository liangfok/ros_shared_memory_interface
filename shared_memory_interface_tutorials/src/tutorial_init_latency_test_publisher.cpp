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
#include <std_msgs/Float64.h>

#include <iostream>

#define WRITE_TO_ROS_TOPIC false
#define NUM_TRANSMIT_TIMES 1
#define PRE_PUBLISH false // must match value in tutorial_init_latency_test_subscriber

// Declare the messages to transmit
std_msgs::Float64 msg1, msg2, msg3;

// Declare the publishers
shared_memory_interface::Publisher<std_msgs::Float64> pub1(WRITE_TO_ROS_TOPIC);
shared_memory_interface::Publisher<std_msgs::Float64> pub2(WRITE_TO_ROS_TOPIC);
shared_memory_interface::Publisher<std_msgs::Float64> pub3(WRITE_TO_ROS_TOPIC);

bool publishMsgs()
{
    if (!pub1.publish(msg1))
    {
         ROS_ERROR("Publisher: Failed to publish message on /topic1. Aborting.");
         return false;
    }

    if (!pub2.publish(msg2))
    {
         ROS_ERROR("Publisher: Failed to publish message on /topic2. Aborting.");
         return false;
    }

    if (!pub3.publish(msg3))
    {
         ROS_ERROR("Publisher: Failed to publish message on /topic3. Aborting.");
         return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    pub1.advertise("/topic1");
    pub2.advertise("/topic2");
    pub3.advertise("/topic3");

    if (PRE_PUBLISH)
    {
        ROS_INFO("Publisher: Pre-publishing messages...");
        if (!publishMsgs())
            return -1;
    }
    else
    {
        ROS_INFO("Publisher: NOT pre-publishing messages...");
    }

    // Wait for user input
    ROS_INFO("Press any key to start publishing...");
    getchar();

    double loopCount = 0;
    int numTransmitTimes = 0;

    ros::Rate loop_rate(1000);
    while (ros::ok()  && numTransmitTimes < NUM_TRANSMIT_TIMES)
    {
        msg1.data = msg2.data = msg3.data = loopCount++;
    
        if (numTransmitTimes == 0)
        {
            char buffer[30];
            struct timeval tv;
            time_t curtime;
      
            gettimeofday(&tv, NULL); 
            curtime = tv.tv_sec;
                    
            strftime(buffer,30,"%m-%d-%Y  %T.", localtime(&curtime));
      
            ROS_INFO_STREAM("Publisher: Starting to publish at time " << buffer << tv.tv_usec);
        }
    
        if (!publishMsgs())
            break;

        numTransmitTimes++;
        loop_rate.sleep();
    }
  
    ROS_INFO("Publisher: Done, press ctrl+c to exit...");
    ros::spin();
    return 0;
}