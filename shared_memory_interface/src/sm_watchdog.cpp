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

#include "shared_memory_interface/sm_watchdog.hpp"

std::string g_interface_name;

void destroySharedMemory(int param)
{
  shared_memory_interface::SharedMemoryInterface::destroyMemory(g_interface_name);
  ros::shutdown();
}

namespace shared_memory_interface
{
  SMWatchdog::SMWatchdog(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
    m_nh.param("interface_name", g_interface_name, std::string("smi"));
  }

  SMWatchdog::~SMWatchdog()
  {
    destroySharedMemory(0);
  }

  void SMWatchdog::spin()
  {
    ROS_INFO("SMWatchdog started.");
    ros::Rate loop_rate(m_loop_rate);
    while(ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_watchdog");
  ros::NodeHandle nh("~");

  shared_memory_interface::SMWatchdog node(nh);

  signal(SIGABRT, destroySharedMemory);
  signal(SIGFPE, destroySharedMemory);
  signal(SIGILL, destroySharedMemory);
  signal(SIGINT, destroySharedMemory);
  signal(SIGSEGV, destroySharedMemory);
  signal(SIGTERM, destroySharedMemory);

  node.spin();

  return 0;
}
