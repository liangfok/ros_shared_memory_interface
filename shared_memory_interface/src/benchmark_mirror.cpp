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

using namespace shared_memory_interface;

SharedMemoryInterfaceROS* smi;
ros::Publisher rtt_pub;
ros::Publisher rosmsg_pub;
ros::Subscriber rosmsg_sub;

void loopbackCallbackROSMsg(const BenchmarkConstPtr& msg)
{
//  ros::Duration(0.1).sleep();
  rosmsg_pub.publish(msg);
}

void loopbackCallbackSMMsg(Benchmark& msg)
{
  smi->publishSerializedROS<Benchmark>("loopback", msg);
}

void loopbackCallbackSMOptimized(std::vector<double>& msg)
{
  smi->publishFloatingPointVector("loopback", msg);
}

int main(int argc, char **argv)
{
  //read arguments
  if(argc != 3 && argc != 5) //roslaunch adds two arguments to the end
  {
    std::cerr << argc - 1 << " arguments specified! Need two arguments: transport ('smoptimized', 'smmsg', 'rosmsg') and data size";
    return 0;
  }
  std::string transport = std::string(argv[1]);
  unsigned long data_size = atoi(argv[2]);

  //set up ros
  ros::init(argc, argv, "echo");
  ros::NodeHandle nh;

  //configure desired transport
  if(transport == "smoptimized")
  {
    smi = new SharedMemoryInterfaceROS(nh.getNamespace());
    smi->advertiseFloatingPointVector("loopback", data_size);
    smi->subscribeFloatingPointVector("loopforward", boost::bind(&loopbackCallbackSMOptimized, _1));
  }
  else if(transport == "smmsg")
  {
    smi = new SharedMemoryInterfaceROS(nh.getNamespace());
    smi->advertiseSerializedROS<Benchmark>("loopback");
    smi->subscribeSerializedROS<Benchmark>("loopforward", boost::bind(&loopbackCallbackSMMsg, _1));
  }
  else if(transport == "rosmsg")
  {
    rosmsg_pub = nh.advertise<Benchmark>("loopback", 1);
    rosmsg_sub = nh.subscribe<Benchmark>("loopforward", 1, &loopbackCallbackROSMsg);
  }
  else
  {
    std::cerr << "Invalid transport specification! Must be 'smoptimized', 'smmsg', or 'rosmsg'.";
    return 0;
  }

  ros::spin();

  delete smi;

  return 0;
}

