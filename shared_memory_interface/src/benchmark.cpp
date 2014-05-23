/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Joshua James
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

#include <ros/ros.h>
#include "shared_memory_interface/shared_memory_interface_ros.hpp"
#include "shared_memory_interface/Benchmark.h"
#include <unistd.h>
#include <string.h>
#include <boost/thread/thread_time.hpp>
#include <std_msgs/Float64.h>

using namespace shared_memory_interface;

bool loopback_complete;
SharedMemoryInterfaceROS* smi;
ros::Publisher rtt_pub;
ros::Publisher rosmsg_pub;
ros::Subscriber rosmsg_sub;

boost::posix_time::ptime send_time;
ros::Time send_time_ros;
void doBenchmark()
{
  boost::posix_time::ptime received_time = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration diff = received_time - send_time;
  std_msgs::Float64 msg;
  msg.data = diff.total_microseconds() * 1.0e-6 - 0.1;
//  msg.data = (ros::Time::now() - send_time_ros).toSec() - 0.1;
  rtt_pub.publish(msg);
  loopback_complete = true;
}

void loopbackCallbackROSMsg(const BenchmarkConstPtr& msg)
{
  doBenchmark();
}

void loopbackCallbackSMMsg(Benchmark& msg)
{
  doBenchmark();
}

void loopbackCallbackSMOptimized(std::vector<double>& msg)
{
  doBenchmark();
}

#define TIMEOUT 1.0
bool timeout()
{
  boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration diff = current_time - send_time;
  if((diff.total_nanoseconds() * 1e-9) > TIMEOUT)
  {
    ROS_WARN("Loopback took longer than a second, trying again");
    return true;
  }
  else
  {
    return false;
  }
}

void waitForCallback()
{
  while(!loopback_complete && ros::ok() && !timeout())
  {
    ros::spinOnce();
  }
}

int main(int argc, char **argv)
{
  //read arguments
  if(argc != 4 && argc != 6) //roslaunch adds two arguments to the end
  {
    std::cerr << argc - 1 << " arguments specified! Need three arguments: transport ('smoptimized', 'smmsg', 'rosmsg'), data size, and nominal rate";
    return 0;
  }
  std::string transport = std::string(argv[1]);
  unsigned long data_size = atoi(argv[2]);
  double nominal_loop_rate = atof(argv[3]);

  //initialize data to random values
  Benchmark msg;
  msg.data.resize(data_size);
  for(unsigned int i = 0; i < data_size; i++)
  {
    msg.data[i] = ((double) rand()) / ((double) RAND_MAX);
  }

  //set up ros
  ros::init(argc, argv, "benchmark");
  ros::NodeHandle nh;
  ros::Rate rate(nominal_loop_rate);
  rtt_pub = nh.advertise<std_msgs::Float64>("rtt", 1);

  //configure desired transport
  if(transport == "smoptimized")
  {
    smi = new SharedMemoryInterfaceROS(nh.getNamespace());
    ROS_INFO("Testing SM optimized transport of %d doubles at %ghz", (int) data_size, nominal_loop_rate);
    smi->advertiseFloatingPointVector("loopforward", data_size);
    smi->subscribeFloatingPointVector("loopback", boost::bind(&loopbackCallbackSMOptimized, _1));
  }
  else if(transport == "smmsg")
  {
    smi = new SharedMemoryInterfaceROS(nh.getNamespace());
    ROS_INFO("Testing SM message transport of %d doubles at %ghz", (int) data_size, nominal_loop_rate);
    smi->advertiseSerializedROS<Benchmark>("loopforward");
    smi->subscribeSerializedROS<Benchmark>("loopback", boost::bind(&loopbackCallbackSMMsg, _1));
  }
  else if(transport == "rosmsg")
  {
    ROS_INFO("Testing ROS message transport of %d doubles at %ghz", (int) data_size, nominal_loop_rate);
    rosmsg_pub = nh.advertise<Benchmark>("loopforward", 1);
    rosmsg_sub = nh.subscribe<Benchmark>("loopback", 1, &loopbackCallbackROSMsg);
  }
  else
  {
    std::cerr << "Invalid transport specification! Must be 'smoptimized', 'smmsg', or 'rosmsg'.";
    return 0;
  }

  std::cerr << "size == " << msg.data.size() << std::endl;

  while(ros::ok())
  {
    loopback_complete = false;
    send_time = boost::posix_time::microsec_clock::local_time();
    send_time_ros = ros::Time::now();
    if(transport == "smoptimized")
    {
      smi->publishFloatingPointVector("loopforward", msg.data);
    }
    else if(transport == "smmsg")
    {
      smi->publishSerializedROS<Benchmark>("loopforward", msg);
    }
    else
    {
      rosmsg_pub.publish(msg);
    }
    waitForCallback();
    rate.sleep();
  }

  delete smi;

  return 0;
}

