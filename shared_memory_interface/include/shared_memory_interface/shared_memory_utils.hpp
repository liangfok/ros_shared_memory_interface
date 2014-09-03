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

#ifndef SHARED_MEMORY_UTILS_H
#define SHARED_MEMORY_UTILS_H

#include <unistd.h>
#include <pwd.h>

namespace shared_memory_interface
{
  typedef boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> SMCharAllocator;
  typedef boost::interprocess::basic_string<char, std::char_traits<char>, SMCharAllocator> SMString;


  inline void getUserUniqueInterfaceName(std::string interface_name, std::string& full_interface_name)
  {
    struct passwd *pass = getpwuid(getuid());
    std::string username = std::string(pass->pw_name);
    full_interface_name = username + "-" + interface_name;
  }

  inline void configureTopicPaths(std::string interface_name, std::string topic_name, std::string& full_ros_topic_path, std::string& full_topic_path, bool use_ros_remapping = true)
  {
    if(use_ros_remapping)
    {
      ros::NodeHandle nh("~");
      full_topic_path = nh.resolveName(topic_name).substr(1); //substr to chop off leading slash
    }
    else
    {
      full_topic_path = topic_name;
    }
    full_ros_topic_path = "/" + interface_name + "/" + full_topic_path;
    std::replace(full_topic_path.begin(), full_topic_path.end(), '/', '-'); //convert slashes for boost compatibility
  }
}

#endif //SHARED_MEMORY_UTILS_H
