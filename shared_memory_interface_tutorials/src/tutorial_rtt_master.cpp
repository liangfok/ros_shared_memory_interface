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

#include "ros/ros.h"
#include "shared_memory_interface/shared_memory_publisher.hpp"
#include "shared_memory_interface/shared_memory_subscriber.hpp"
#include <std_msgs/Int64.h>


int rcvdCount;


void rttRxCallback(std_msgs::Int64& msg)
{
  rcvdCount = msg.data;
  ROS_INFO("Master: Received %i", rcvdCount);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master");
  ros::NodeHandle n;

  shared_memory_interface::Subscriber<std_msgs::Int64> sub;
  sub.subscribe("/rtt_rx", boost::bind(&rttRxCallback, _1));

  shared_memory_interface::Publisher<std_msgs::Int64> pub;
  pub.advertise("/rtt_tx");

  ros::Rate loop_rate(1000);
  int count = 0;
  rcvdCount = count;  // set rcvdCount equal to count to trigger send

  std_msgs::Int64 msg;

  while (ros::ok())
  {
    // ROS_INFO("Master: begin loop cycle.");
    if (rcvdCount == count)
    {
      msg.data = ++count;
      ROS_INFO("Master: publishing %i", msg.data);
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
