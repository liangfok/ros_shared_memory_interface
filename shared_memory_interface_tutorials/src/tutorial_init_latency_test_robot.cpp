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

#define LISTEN_TO_ROS_TOPIC false
#define USE_POLLING false
#define WRITE_TO_ROS_TOPIC false
#define NUM_TRANSMIT_TIMES 1000
#define PRE_PUBLISH false // must match value in tutorial_init_latency_test_controller

// Declare the messages to transmit
std_msgs::Float64 msg1, msg2, msg3;

// Declare the publishers
shared_memory_interface::Publisher<std_msgs::Float64> pub1(WRITE_TO_ROS_TOPIC);
shared_memory_interface::Publisher<std_msgs::Float64> pub2(WRITE_TO_ROS_TOPIC);
shared_memory_interface::Publisher<std_msgs::Float64> pub3(WRITE_TO_ROS_TOPIC);

// Declare variables for holding receive state
int rcvCnt1 = 0, rcvCnt2 = 0;
bool rcvd1 = false, rcvd2 = false;
struct timeval tv1, tv2;
double data1, data2;

int doneRcvCnt = 1;

static void printRcvTime(std::string callbackName, struct timeval & tv, double value)
{
    char buffer[30];
    time_t curtime;
    curtime = tv.tv_sec;
    strftime(buffer, 30, "%m-%d-%Y  %T.", localtime(&curtime));
    ROS_INFO_STREAM("Robot: " << callbackName << " called, time = " << buffer << tv.tv_usec << ", value = " << value);
}

bool publishMsgs()
{
    if (!pub1.publish(msg1))
    {
        ROS_ERROR("Robot: Failed to publish message on /topic1. Aborting.");
        return false;
    }

    if (!pub2.publish(msg2))
    {
        ROS_ERROR("Robot: Failed to publish message on /topic2. Aborting.");
        return false;
    }

    if (!pub3.publish(msg3))
    {
        ROS_ERROR("Robot: Failed to publish message on /topic3. Aborting.");
        return false;
    }

    return true;
}

void callback1(std_msgs::Float64& msg)
{
    if (++rcvCnt1 == doneRcvCnt)
    {
        gettimeofday(&tv1, NULL);
        data1 = msg.data;
        rcvd1 = true;
    }
}

void callback2(std_msgs::Float64& msg)
{
    if (++rcvCnt2 == doneRcvCnt)
    {
        gettimeofday(&tv2, NULL);
        data2 = msg.data;
        rcvd2 = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // Initialize the publishers
    pub1.advertise("/robot1");
    pub2.advertise("/robot2");
    pub3.advertise("/robot3");

    // Declare three subscribers
    shared_memory_interface::Subscriber<std_msgs::Float64> sub1(LISTEN_TO_ROS_TOPIC, USE_POLLING);
    shared_memory_interface::Subscriber<std_msgs::Float64> sub2(LISTEN_TO_ROS_TOPIC, USE_POLLING);

    sub1.subscribe("/controller1", boost::bind(&callback1, _1));
    sub2.subscribe("/controller2", boost::bind(&callback2, _1));

    if (PRE_PUBLISH)
    {
        doneRcvCnt = 2;
        ROS_INFO("Robot: Pre-publishing messages...");
        if (!publishMsgs())
            return -1;
    }
    else
    {
        ROS_INFO("Robot: NOT pre-publishing messages...");
    }

    // Wait for user input
    ROS_INFO("Robot: Press any key to start publishing...");
    getchar();

    int loopCount = 0;
    int numTransmitTimes = 0;

    ros::Rate loop_rate(1000);
    while (ros::ok()  && 
        (numTransmitTimes < NUM_TRANSMIT_TIMES || !rcvd1 || !rcvd2))
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
      
            ROS_INFO_STREAM("Robot: Starting to publish at time " << buffer << tv.tv_usec);
        }
    
        if (!publishMsgs())
            break;

        numTransmitTimes++;
        loop_rate.sleep();
    }

    printRcvTime("callback1", tv1, data1);
    printRcvTime("callback2", tv2, data2);
  
    ROS_INFO("Robot: Done, press ctrl+c to exit...");
    ros::spin();
    return 0;
}