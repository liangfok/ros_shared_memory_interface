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

//
//  class SMScopedLock
//  {
//  public:
//    SMScopedLock(std::string interface_name, std::string field_name)
//    {
//      assert(interface_name.length() != 0);
//      m_interface_name = interface_name;
//      m_field_name = field_name;
//      m_full_name = getFullName(m_interface_name, m_field_name);
//      try
//      {
//        m_mutex = new boost::interprocess::named_upgradable_mutex(boost::interprocess::open_only, m_full_name.c_str());
//      }
//      catch(...)
//      {
//        std::cerr << "Mutex " << m_full_name << " not found! Creating it!";
//        m_mutex = new boost::interprocess::named_upgradable_mutex(boost::interprocess::open_or_create, m_full_name.c_str(), unrestricted());
//      }
//    }
//
//    ~SMScopedLock()
//    {
//      delete m_mutex;
//    }
//
//    boost::interprocess::named_upgradable_mutex* get()
//    {
//      return m_mutex;
//    }
//
//    static void destroy(std::string interface_name, std::string field_name)
//    {
//      assert(interface_name.length() != 0);
//      boost::interprocess::named_upgradable_mutex::remove(getFullName(interface_name, field_name).c_str());
//    }
//
//    static void create(std::string interface_name, std::string field_name)
//    {
//      assert(interface_name.length() != 0);
//      boost::interprocess::named_upgradable_mutex(boost::interprocess::open_or_create, getFullName(interface_name, field_name).c_str(), unrestricted());
//    }
//
//    static std::string getFullName(std::string& interface_name, std::string field_name)
//    {
//      assert(interface_name.length() != 0);
//      //std::cerr << interface_name << "+" << field_name << "+mutex=" << (interface_name + field_name + "mutex") << std::endl;
//      return (interface_name + "__" + field_name + "__mutex");
//    }
//
//  protected:
//    void repairBrokenLock()
//    {
//      std::cerr << "SharedMemoryTransport: Found broken lock " << m_full_name << "! Repairing!" << std::endl;
//      assert(m_interface_name.length() != 0);
//      destroy(m_interface_name, m_field_name);
//      create(m_interface_name, m_field_name);
//      delete m_mutex;
//      m_mutex = new boost::interprocess::named_upgradable_mutex(boost::interprocess::open_only, m_full_name.c_str());
//    }
//
//    boost::interprocess::named_upgradable_mutex* m_mutex;
//    std::string m_interface_name;
//    std::string m_field_name;
//    std::string m_full_name;
//  };
//
//  class SMScopedReaderLock: public SMScopedLock
//  {
//  public:
//    SMScopedReaderLock(std::string interface_name, std::string field_name, double timeout_duration = 1.0) :
//        SMScopedLock(interface_name, field_name)
//    {
//      boost::posix_time::ptime timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout_duration * 1000);
//      if(!m_mutex->timed_lock_sharable(timeout))
//      {
//        repairBrokenLock();
//      }
//    }
//
//    ~SMScopedReaderLock()
//    {
//      m_mutex->unlock_sharable();
//    }
//  };
//
//  class SMScopedWriterLock: public SMScopedLock
//  {
//  public:
//    SMScopedWriterLock(std::string interface_name, std::string field_name, double timeout_duration = 1.0) :
//        SMScopedLock(interface_name, field_name)
//    {
//      boost::posix_time::ptime timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout_duration * 1000);
//      if(!m_mutex->timed_lock(timeout))
//      {
//        repairBrokenLock();
//      }
//    }
//
//    ~SMScopedWriterLock()
//    {
//      m_mutex->unlock();
//    }
//  };
}

#endif //SHARED_MEMORY_UTILS_H
