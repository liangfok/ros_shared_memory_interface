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

#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/parameter_adapter.h>
#include <ros/subscription_callback_helper.h>
#include <ros/message_deserializer.h>
#include <ros/package.h>

#include <vector>
#include <stdio.h>
#include <algorithm>
#include <stdlib.h>

#include <unistd.h>
#include <pwd.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

#include <boost/interprocess/exceptions.hpp>
#include <boost/thread/thread_time.hpp>

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/type_traits.hpp>

namespace shared_memory_interface
{
#define TRACE 0
#if TRACE
#define PRINT_TRACE_ENTER #if(TRACE) std::cerr<<__func__<<std::endl;
#define PRINT_TRACE_EXIT #if(TRACE) std::cerr<<"/"<<__func__<<std::endl;
#else
#define PRINT_TRACE_ENTER
#define PRINT_TRACE_EXIT
#endif

#define ROS_ID_DEBUG_THROTTLED_STREAM(...) ROS_DEBUG_STREAM_THROTTLE(1.0, "SharedMemoryTransport (" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_INFO_THROTTLED_STREAM(...) ROS_INFO_STREAM_THROTTLE(1.0, "SharedMemoryTransport (" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_WARN_THROTTLED_STREAM(...) ROS_WARN_STREAM_THROTTLE(1.0, "SharedMemoryTransport (" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_ERROR_THROTTLED_STREAM(...) ROS_ERROR_STREAM_THROTTLE(1.0, "SharedMemoryTransport (" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_DEBUG_STREAM(...) ROS_DEBUG_STREAM("SharedMemoryTransport (" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_INFO_STREAM(...) ROS_INFO_STREAM("SharedMemoryTransport (" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_WARN_STREAM(...) ROS_WARN_STREAM("SharedMemoryTransport (" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_ERROR_STREAM(...) ROS_ERROR_STREAM("SharedMemoryTransport (" << getpid() << "): "<<__VA_ARGS__)

#define TEST_INITIALIZED if(!m_initialized) {ROS_ID_ERROR_STREAM("Tried to call " <<__func__ << " on an uninitialized shared memory transport!"); return false;}
#define TEST_CONNECTED if(!m_connected) {ROS_ID_ERROR_STREAM("Tried to call " <<__func__ << " on an unconnected shared memory transport!"); return false;}
#define CATCH_SHUTDOWN_SIGNAL if(!m_initialized) {ROS_ID_DEBUG_STREAM("Caught shutdown signal in function " <<__func__ << "!"); return false;}

  typedef boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> SMCharAllocator;
  typedef boost::interprocess::basic_string<char, std::char_traits<char>, SMCharAllocator> SMString;

  boost::interprocess::permissions unrestricted()
  {
    boost::interprocess::permissions perm;
    perm.set_unrestricted();
    return perm;
  }

  bool createMemory(std::string interface_name, unsigned int size)
  {
    PRINT_TRACE_ENTER
    //make sure the system will let us create a memory space of the desired size
    std::ifstream shmmax_file_read;
    shmmax_file_read.open("/proc/sys/kernel/shmmax");
    if(!shmmax_file_read.is_open())
    {
      ROS_ID_ERROR_STREAM("System shared memory maximum file not found at /proc/sys/kernel/shmmax. System may or may not have sufficient shared memory available. Are you using Ubuntu 12.04?");
    }
    else
    {
      std::string shmmax_string;
      std::getline(shmmax_file_read, shmmax_string);
      unsigned int shmmax = atof(shmmax_string.c_str());
      shmmax_file_read.close();
      if(shmmax < size)
      {
        ROS_ID_WARN_STREAM("Available system shared memory (" << shmmax << ") was smaller than the requested size (" << size << ")! Attempting to increase it (may need sudo). To make the change persist across reboots: `rosrun shared_memory_interface set_shared_memory_size_persistent " << size << "`");

        std::stringstream ss;
        ss << ros::package::getPath("shared_memory_interface") << "/scripts/set_shared_memory_size " << size;
        int unused = system(ss.str().c_str());
        unused = unused; //silly warnings are silly

        //check to see if we actually changed it
        shmmax_file_read.open("/proc/sys/kernel/shmmax");
        std::getline(shmmax_file_read, shmmax_string);
        shmmax = atof(shmmax_string.c_str());
        shmmax_file_read.close();

        if(shmmax == size)
        {
          ROS_ID_INFO_STREAM("Successfully increased system shared memory!");
        }
        else
        {
          ROS_ID_WARN_STREAM("Failed to increase system shared memory, and will continue using the current maximum " << shmmax << "! If you really need more space, you may need to increase it manually (`rosrun shared_memory_interface set_shared_memory_size_persistent " << size << "` or `rosrun shared_memory_interface set_shared_memory_size " << size << ").");
          size = shmmax;
        }
      }
    }

    //try to create the memory space
    try
    {
      ROS_ID_INFO_STREAM("Creating shared memory space " << interface_name << "..");
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::create_only, interface_name.c_str(), size, NULL, unrestricted());
      ROS_ID_INFO_STREAM("Created " << interface_name << " space!");

      segment.construct<bool>("shutdown_required")(false);
    }
    catch(boost::interprocess::interprocess_exception &ex) //shared memory hasn't been created yet, so we'll make it
    {
      ROS_ID_WARN_STREAM("Shared memory space already existed!");
      return false;
    }
    PRINT_TRACE_EXIT
    return true;
  }

  //cerr used below because ROS doesn't work after ros::shutdown has happened.
  void destroyMemory(std::string interface_name)
  {
    PRINT_TRACE_ENTER
    std::cerr << "SharedMemoryTransport(" << getpid() << "): " << "Destroying shared memory space " << interface_name << "..." << std::endl;
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());
      boost::interprocess::shared_memory_object::remove(interface_name.c_str());
      *segment.find<bool>("shutdown_required").first = true; //inform the other processes that the shared memory needs to close
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "Exception occurred during destroyMemory: " << ex.what() << std::endl;
    }
    boost::interprocess::shared_memory_object::remove(interface_name.c_str());
    //NOTE: shared memory will be unlinked, not destroyed. Anyone who already has the space mapped will still be able to run,
    //but new processes will not be able to open the space again. The memory space will only be destroyed when the last
    //managed_shared_memory object is destroyed.
    usleep(2000000);
    boost::interprocess::shared_memory_object::remove(interface_name.c_str());
    std::cerr << "SharedMemoryTransport(" << getpid() << "): " << "Shared memory space successfully destroyed." << std::endl;
    PRINT_TRACE_EXIT
  }

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
