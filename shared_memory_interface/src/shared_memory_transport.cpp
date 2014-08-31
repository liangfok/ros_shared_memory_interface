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

#include "shared_memory_interface/shared_memory_transport.hpp"

namespace shared_memory_interface
{
#define TRACE 0
#define PRINT_TRACE_ENTER if(TRACE)std::cerr<<__func__<<std::endl;
#define PRINT_TRACE_EXIT if(TRACE)std::cerr<<"/"<<__func__<<std::endl;

  boost::interprocess::permissions unrestricted()
  {
    boost::interprocess::permissions perm;
    perm.set_unrestricted();
    return perm;
  }

  void SharedMemoryTransport::createMemory(std::string interface_name, unsigned int size)
  {
    //make sure the system will let us create a memory space of the desired size
    std::ifstream shmmax_file_read;
    shmmax_file_read.open("/proc/sys/kernel/shmmax");
    if(!shmmax_file_read.is_open())
    {
      std::cerr << "SharedMemoryTransport: ERROR: System shared memory maximum file not found at /proc/sys/kernel/shmmax. System may or may not have sufficient shared memory available. Are you using Ubuntu 12.04?" << std::endl;
    }
    std::string shmmax_string;
    std::getline(shmmax_file_read, shmmax_string);
    shmmax_file_read.close();
    unsigned int shmmax = atof(shmmax_string.c_str());
    if(shmmax < size)
    {
      std::cerr << "SharedMemoryTransport: Available system shared memory was too small. Attempting to correct...";
      std::ofstream shmmax_file_write;
      shmmax_file_write.open("/proc/sys/kernel/shmmax");
      shmmax_file_write << size;
      shmmax_file_write.close();
    }

    //try to create the memory space
    try
    {
      std::cerr << "SharedMemoryTransport: Creating shared memory space " << interface_name << ".." << std::endl;
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::create_only, interface_name.c_str(), size, NULL, unrestricted());
      std::cerr << "SharedMemoryTransport: Created " << interface_name << " space!" << std::endl;
    }
    catch(boost::interprocess::interprocess_exception &ex) //shared memory hasn't been created yet, so we'll make it
    {
      std::cerr << "SharedMemoryTransport: ERROR: shared memory space already existed!";
    }
  }

  void SharedMemoryTransport::destroyMemory(std::string interface_name)
  {
    std::cerr << "SharedMemoryTransport: Destroying shared memory space " << interface_name << "." << std::endl;
    boost::interprocess::shared_memory_object::remove(interface_name.c_str());
  }

  SharedMemoryTransport::SharedMemoryTransport()
  {
    m_initialized = false;
  }

  SharedMemoryTransport::~SharedMemoryTransport()
  {
  }

  void SharedMemoryTransport::configure(std::string interface_name, std::string field_name)
  {
    while(ros::ok()) //there's probably a much less silly way to do this...
    {
      try
      {
        segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());
        std::cerr << "SharedMemoryTransport: Connected to " << interface_name << " space." << std::endl;
        break;
      }
      catch(boost::interprocess::interprocess_exception &ex) //shared memory hasn't been created yet, so we'll make it
      {
        std::cerr << "SharedMemoryTransport: Waiting for shared memory space " << interface_name << " to become available (is the manager running?)..." << std::endl;
      }
    }

    m_field_name = field_name;
    m_buffer_0_name = m_field_name + "_0";
    m_buffer_1_name = m_field_name + "_1";
    m_buffer_selector_name = m_field_name + "_buffer_selector";
    m_invalid_flag_name = m_field_name + "_invalid";
    m_condition_name = m_field_name + "_condition";
    m_condition_mutex_name = m_field_name + "_condition_mutex";
    m_initialized = true; //TODO: check names initialized everywhere
  }

  bool SharedMemoryTransport::createField()
  {
    PRINT_TRACE_ENTER
    if(fieldExists()) //check to see if someone else created the field
    {
      PRINT_TRACE_EXIT
      return true;
    }

    try
    {
      std::cerr << "Creating new shared memory field for " << m_field_name << std::endl;
      segment.construct<SMString>(m_buffer_0_name.c_str())(segment.get_segment_manager());
      segment.construct<SMString>(m_buffer_1_name.c_str())(segment.get_segment_manager());

      segment.construct<bool>(m_buffer_selector_name.c_str())(false);
      segment.construct<bool>(m_invalid_flag_name.c_str())(true); //field is invalid until someone writes actual data to it

//      segment.construct<boost::interprocess::interprocess_condition>(m_condition_name.c_str())(segment.get_segment_manager());
//      segment.construct<boost::interprocess::interprocess_mutex>(m_condition_mutex_name.c_str())(segment.get_segment_manager());
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while creating new field \"" << m_field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getData(std::string& data)
  {
    PRINT_TRACE_ENTER
    if(!hasData())
    {
      PRINT_TRACE_EXIT
      return false;
    }

    while(ros::ok())
    {
      bool buffer_selector = *segment.find<bool>(m_buffer_selector_name.c_str()).first;
      std::string read_buffer_name;
      if(buffer_selector)
      {
        read_buffer_name = m_buffer_0_name;
      }
      else
      {
        read_buffer_name = m_buffer_1_name;
      }

      SMString* field_data = segment.find<SMString>(read_buffer_name.c_str()).first;

      if(buffer_selector != *segment.find<bool>(m_buffer_selector_name.c_str()).first)
      {
        //someone wrote to the buffer while we were reading it! try again...
        //TODO: add counter to detect starvation?
        continue;
      }

      data = std::string(field_data->begin(), field_data->end());

      PRINT_TRACE_EXIT
      return true;
    }

    PRINT_TRACE_EXIT
    return false;
  }

  bool SharedMemoryTransport::setData(std::string data)
  {
    PRINT_TRACE_ENTER
    if(!m_initialized || !fieldExists())
    {
      PRINT_TRACE_EXIT
      return false;
    }

    bool* buffer_selector = segment.find<bool>(m_buffer_selector_name.c_str()).first;
    std::string write_buffer_name = (*buffer_selector)? m_buffer_1_name : m_buffer_0_name;
    *segment.find<SMString>(write_buffer_name.c_str()).first = SMString(data.begin(), data.end(), segment.get_segment_manager());
    *segment.find<bool>(m_invalid_flag_name.c_str()).first = false;
    *buffer_selector = !(*buffer_selector);
//    segment.find<boost::interprocess::interprocess_condition>(m_condition_name.c_str()).first->notify_all();

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::fieldExists()
  {
    return m_initialized && ((segment.find<SMString>(m_buffer_0_name.c_str()).first != NULL) && (segment.find<SMString>(m_buffer_1_name.c_str()).first != NULL));
  }

  bool SharedMemoryTransport::hasData()
  {
    return fieldExists() && !(*segment.find<bool>(m_invalid_flag_name.c_str()).first);
  }

  bool SharedMemoryTransport::awaitNewDataPolled(std::string& data, double timeout)
  {
    PRINT_TRACE_ENTER
    while(!hasData()) //wait for the field to at least have something
    {
      ROS_DEBUG_THROTTLE(1.0, "Waiting for field %s to become valid.", m_field_name.c_str());
      boost::this_thread::interruption_point();
    }

    if(timeout == 0)
    {
      PRINT_TRACE_EXIT
      return getData(data);
    }
    else
    {
      bool initial_buffer_selector = *segment.find<bool>(m_buffer_selector_name.c_str()).first;
      boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
      while(ros::ok() && (initial_buffer_selector == *segment.find<bool>(m_buffer_selector_name.c_str()).first)) //wait for the selector to change
      {
        if(timeout < 0)
        {
          ROS_DEBUG_THROTTLE(1.0, "Waiting for new data in field %s.", m_field_name.c_str());
        }
        else if(boost::get_system_time() < timeout_time)
        {
          ROS_DEBUG_THROTTLE(1.0, "Waiting for new data in field %s with timeout %g.", m_field_name.c_str(), timeout);
        }
        else
        {
          ROS_DEBUG_THROTTLE(1.0, "Timed out while waiting for new data in field %s with timeout %g!", m_field_name.c_str(), timeout);
          return false;
        }
        boost::this_thread::interruption_point();
      }

      return getData(data);
    }
  }

  bool SharedMemoryTransport::awaitNewData(std::string& data, double timeout)
  {
    PRINT_TRACE_ENTER

    ROS_ERROR("AWAIT NEW DATA DOESN'T WORK YET! USE THE POLLED VERSION FOR NOW!");
    PRINT_TRACE_EXIT
    return false;

//    if(timeout == 0)
//    {
//      PRINT_TRACE_EXIT
//      return hasData();
//    }
//
//    boost::interprocess::interprocess_condition* condition = segment.find<boost::interprocess::interprocess_condition>(m_condition_name.c_str()).first;
//    boost::interprocess::interprocess_mutex* mutex = segment.find<boost::interprocess::interprocess_mutex>(m_condition_mutex_name.c_str()).first;
//    (*mutex).lock();
//    if(timeout < 0)
//    {
//      condition->wait(*mutex);
//    }
//    else
//    {
//      boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
//      if(!condition->timed_wait(*mutex, timeout_time))
//      {
//        PRINT_TRACE_EXIT
//        return false;
//      }
//    }
//    PRINT_TRACE_EXIT
//    return true;
  }
}
