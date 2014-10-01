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

#include "shared_memory_interface/shared_memory_manager.hpp"
#include <stdio.h>
#include <pwd.h>

namespace shared_memory_interface
{
  SharedMemoryManager::SharedMemoryManager(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
    m_nh.param("interface_name", m_interface_name, std::string("smi"));
    m_nh.param("memory_size", m_memory_size, 512.0 * 1024.0 * 1024.0); //param is double because ros apparently doesn't like unsigned int
    m_nh.param("force_kill", m_force_kill, false);
    if(m_force_kill)
    {
      while(ros::ok())
      {
        SharedMemoryTransport::destroyMemory(m_interface_name);
      }
    }
    else
    {
      if(!SharedMemoryTransport::createMemory(m_interface_name, (unsigned int) m_memory_size))
      {
        ROS_WARN("Another shared_memory_manager appears to be running. Shutting down!");
        ros::shutdown();
        m_memory_created = false;
        return;
      }
    }
    m_memory_created = true;
  }

  SharedMemoryManager::~SharedMemoryManager()
  {
    if(m_memory_created)
    {
      SharedMemoryTransport::destroyMemory(m_interface_name);
    }
  }

  void SharedMemoryManager::spin()
  {
    ROS_INFO("SharedMemoryManager started.");
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
  ros::init(argc, argv, "shared_memory_manager", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  shared_memory_interface::SharedMemoryManager node(nh);
  node.spin();

  return 0;
}
