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
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

#define WRITE_TO_ROS_TOPIC false
#define LISTEN_TO_ROS_TOPIC false
#define USE_POLLING true

int NUM_SAMPLES = 10000; //Default Value
int SIZE_SAMPLES = 1; //Default Value
double *data;

int dataIndex = 0;
bool firstRound = true; // keep track of first round so that we can ignore it
bool roundDone = true;

ros::Time sendTime;

void printStats()
{
  double sum = 0;
  double max = data[0];
  double min = data[0];

  for (int ii = 0; ii < NUM_SAMPLES; ii++)
  {
    sum += data[ii];
    if (max < data[ii]) max = data[ii];
    if (min > data[ii]) min = data[ii];
  }
  double avg = sum / NUM_SAMPLES;

  double variance = 0;
  for (int ii = 0; ii < NUM_SAMPLES; ii++)
  {
    variance += (data[ii] - avg) * (data[ii] - avg);
  }

  variance /= NUM_SAMPLES;
  double stdev = sqrt(variance);

  ROS_INFO_STREAM("RTT Benchmark statistics:\n"
    << " - Num samples: " << NUM_SAMPLES << "\n"
    << " - Size samples: "<<SIZE_SAMPLES << "\n"
    << " - Average (us): " << avg << "\n" 
    << " - Standard deviation: " << stdev << "\n"
    << " - Min (us): " << min << "\n"
    << " - Max (us): " << max);

  delete [] data;
  ros::shutdown();
}

void rttRxCallback(std_msgs::Float64MultiArray& msg)
{
  if (!firstRound && dataIndex < NUM_SAMPLES)
  {
    // Compute the time since the sequence number was sent.    
    double rtt = (ros::Time::now() - sendTime).toSec() * 1e6;

    ROS_INFO("RTT %i: %f us", dataIndex + 1, rtt);
    data[dataIndex++] = rtt;

    if (dataIndex == NUM_SAMPLES)
      printStats();
  }

  firstRound = false;
  roundDone = true; // triggers the sending of the next RTT number
}

int main(int argc, char **argv)
{

  if (argc != 1)
  {
    if (argc == 3)
    {
      // Change the NUM_SAMPLES by reading the argument
      NUM_SAMPLES = atoll(argv[1]);
      SIZE_SAMPLES = atoll(argv[2]);
    }
    else
    {
      std::cout << "Accept TWO argument: NUM_SAMPLES & SIZE_SAMPLES\n"
                << "  - NUM_SAMPLES: The number of round trip times to measure.\n"
                << "  - SIZE_SAMPLES: The number of Float64 values to transmit." 
                << std::endl;
      return 1;
    }
  }
  else
  {
    std::cout<<"Use default value: NUM_SAMPLES: "<<NUM_SAMPLES<<"; SIZE_SAMPLES: "<<SIZE_SAMPLES<<std::endl;
  }

  data = new double[NUM_SAMPLES];

  std_msgs::Float64MultiArray msg;
  std_msgs::MultiArrayDimension dim;
  dim.size = SIZE_SAMPLES;
  dim.stride = SIZE_SAMPLES;
  msg.layout.data_offset = 0;
  msg.layout.dim.push_back(dim);
  msg.data.resize(SIZE_SAMPLES);

  ros::init(argc, argv, "master", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  shared_memory_interface::Publisher<std_msgs::Float64MultiArray> pub(WRITE_TO_ROS_TOPIC);
  pub.advertise("/rtt_tx");

  shared_memory_interface::Subscriber<std_msgs::Float64MultiArray> sub(LISTEN_TO_ROS_TOPIC, USE_POLLING);
  sub.subscribe("/rtt_rx", boost::bind(&rttRxCallback, _1));

  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
    if (roundDone)
    {
      roundDone = false;
      sendTime = ros::Time::now();
      if (!pub.publish(msg))
      {
        ROS_ERROR("Master: Failed to publish message. Aborting.");
        break;
      }
    }
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
