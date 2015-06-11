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
#include "std_msgs/Float64.h"
// #include "std_msgs/Float64MultiArray.h"
// #include "std_msgs/MultiArrayDimension.h"

#define WRITE_TO_ROS_TOPIC false
#define LISTEN_TO_ROS_TOPIC false
#define USE_POLLING true

// int NUM_SAMPLES = 10000; //Default Value
// int SIZE_SAMPLES = 1; //Default Value
// double *data;

// int dataIndex = 0;
// bool firstRound = true; // keep track of first round so that we can ignore it
// bool roundDone = true;

// ros::Time sendTime;

// void printStats()
// {
//   double sum = 0;
//   double max = data[0];
//   double min = data[0];

//   for (int ii = 0; ii < NUM_SAMPLES; ii++)
//   {
//     sum += data[ii];
//     if (max < data[ii]) max = data[ii];
//     if (min > data[ii]) min = data[ii];
//   }
//   double avg = sum / NUM_SAMPLES;

//   double variance = 0;
//   for (int ii = 0; ii < NUM_SAMPLES; ii++)
//   {
//     variance += (data[ii] - avg) * (data[ii] - avg);
//   }

//   variance /= NUM_SAMPLES;
//   double stdev = sqrt(variance);

//   ROS_INFO_STREAM("RTT Benchmark statistics:\n"
//     << " - Num samples: " << NUM_SAMPLES << "\n"
//     << " - Size samples: "<<SIZE_SAMPLES << "\n"
//     << " - Average (us): " << avg << "\n" 
//     << " - Standard deviation: " << stdev << "\n"
//     << " - Min (us): " << min << "\n"
//     << " - Max (us): " << max);

//   delete [] data;
//   ros::shutdown();
// }

bool firstRcv1 = true;
bool firstRcv2 = true;
bool firstRcv3 = true;

static void printRcvTime(std::string callbackName)
{
  char buffer[30];
  struct timeval tv;
  time_t curtime;

  gettimeofday(&tv, NULL); 
  curtime=tv.tv_sec;
          
  strftime(buffer,30,"%m-%d-%Y  %T.", localtime(&curtime));

  ROS_INFO_STREAM(callbackName << " called at time " << buffer << tv.tv_usec);
}

void callback1(std_msgs::Float64& msg)
{
  if (firstRcv1)
  {
    printRcvTime("callback1");
    firstRcv1 = false;
  }
}

void callback2(std_msgs::Float64& msg)
{
  if (firstRcv2)
  {
    printRcvTime("callback2");
    firstRcv2 = false;
  }
}

void callback3(std_msgs::Float64& msg)
{
  if (firstRcv3)
  {
    printRcvTime("callback3");
    firstRcv3 = false;
  }
}

int main(int argc, char **argv)
{

  // if (argc != 1)
  // {
  //   if (argc == 3)
  //   {
  //     // Change the NUM_SAMPLES by reading the argument
  //     NUM_SAMPLES = atoll(argv[1]);
  //     SIZE_SAMPLES = atoll(argv[2]);
  //   }
  //   else
  //   {
  //     std::cout << "Accept TWO argument: NUM_SAMPLES & SIZE_SAMPLES\n"
  //               << "  - NUM_SAMPLES: The number of round trip times to measure.\n"
  //               << "  - SIZE_SAMPLES: The number of Float64 values to transmit." 
  //               << std::endl;
  //     return 1;
  //   }
  // }
  // else
  // {
  //   std::cout<<"Use default value: NUM_SAMPLES: "<<NUM_SAMPLES<<"; SIZE_SAMPLES: "<<SIZE_SAMPLES<<std::endl;
  // }

  // data = new double[NUM_SAMPLES];

  // std_msgs::Float64MultiArray msg;
  // std_msgs::MultiArrayDimension dim;
  // dim.size = SIZE_SAMPLES;
  // dim.stride = SIZE_SAMPLES;
  // msg.layout.data_offset = 0;
  // msg.layout.dim.push_back(dim);
  // msg.data.resize(SIZE_SAMPLES);

  // std_msgs::Float64 msg1, msg2;

  ros::init(argc, argv, "controller", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  // Controller sends on two topics
  // shared_memory_interface::Publisher<std_msgs::Float64> pub1(WRITE_TO_ROS_TOPIC);
  // shared_memory_interface::Publisher<std_msgs::Float64> pub2(WRITE_TO_ROS_TOPIC);
  // pub1.advertise("/controller1");
  // pub2.advertise("/controller2");

  // Robot sends on three topics
  shared_memory_interface::Subscriber<std_msgs::Float64> sub1(LISTEN_TO_ROS_TOPIC, USE_POLLING);
  shared_memory_interface::Subscriber<std_msgs::Float64> sub2(LISTEN_TO_ROS_TOPIC, USE_POLLING);
  shared_memory_interface::Subscriber<std_msgs::Float64> sub3(LISTEN_TO_ROS_TOPIC, USE_POLLING);
  sub1.subscribe("/robot1", boost::bind(&callback1, _1));
  sub1.subscribe("/robot2", boost::bind(&callback2, _1));
  sub1.subscribe("/robot3", boost::bind(&callback3, _1));

  // double loopCount = 0;

  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
    // msg1.data = msg2.data = loopCount++;
    // if (!pub1.publish(msg1))
    // {
    //     ROS_ERROR("Master: Failed to publish message on /controller1. Aborting.");
    //     break;
    // }

    // if (!pub2.publish(msg2))
    // {
    //     ROS_ERROR("Master: Failed to publish message on /controller2. Aborting.");
    //     break;
    // }

    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
