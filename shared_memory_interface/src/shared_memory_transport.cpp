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

#define ROS_ID_DEBUG_THROTTLED_STREAM(...) ROS_DEBUG_STREAM_THROTTLE(1.0, "SharedMemoryTransport(" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_INFO_THROTTLED_STREAM(...) ROS_INFO_STREAM_THROTTLE(1.0, "SharedMemoryTransport(" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_WARN_THROTTLED_STREAM(...) ROS_WARN_STREAM_THROTTLE(1.0, "SharedMemoryTransport(" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_ERROR_THROTTLED_STREAM(...) ROS_ERROR_STREAM_THROTTLE(1.0, "SharedMemoryTransport(" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_DEBUG_STREAM(...) ROS_DEBUG_STREAM("SharedMemoryTransport(" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_INFO_STREAM(...) ROS_INFO_STREAM("SharedMemoryTransport(" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_WARN_STREAM(...) ROS_WARN_STREAM("SharedMemoryTransport(" << getpid() << "): "<<__VA_ARGS__)
#define ROS_ID_ERROR_STREAM(...) ROS_ERROR_STREAM("SharedMemoryTransport(" << getpid() << "): "<<__VA_ARGS__)
#define TEST_INITIALIZED if(!m_initialized) {ROS_ID_ERROR_STREAM("Tried to call " <<__func__ << " on an uninitialized shared memory transport!"); return false;}
#define TEST_CONNECTED if(!m_connected) {ROS_ID_ERROR_STREAM("Tried to call " <<__func__ << " on an unconnected shared memory transport!"); return false;}
#define CATCH_SHUTDOWN_SIGNAL if(!m_initialized) {ROS_ID_DEBUG_STREAM("Caught shutdown signal in function " <<__func__ << "!"); return false;}

  boost::interprocess::permissions unrestricted()
  {
    boost::interprocess::permissions perm;
    perm.set_unrestricted();
    return perm;
  }

  bool SharedMemoryTransport::createMemory(std::string interface_name, unsigned int size)
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
  void SharedMemoryTransport::destroyMemory(std::string interface_name)
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
      std::cerr << ex.what() << std::endl;
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

  SharedMemoryTransport::SharedMemoryTransport()
  {
    m_initialized = false;
    m_watchdog_thread = NULL;
  }

  SharedMemoryTransport::~SharedMemoryTransport()
  {
    if(m_watchdog_thread != NULL)
    {
      m_watchdog_thread->interrupt();
      m_watchdog_thread->detach();
      delete m_watchdog_thread;
    }
  }

  void SharedMemoryTransport::watchdogFunction()
  {
    bool* shutdown_required = NULL;
    while(ros::ok())
    {
      shutdown_required = segment->find<bool>("shutdown_required").first;
      if(!shutdown_required)
      {
        ROS_ID_WARN_THROTTLED_STREAM("Watchdog waiting for shutdown signal field...");
        continue;
      }
      boost::this_thread::interruption_point();
    }

    while(ros::ok())
    {
      if(*shutdown_required)
      {
        m_initialized = false;
        ROS_ID_WARN_STREAM("Shutdown signal detected! Disconnecting from shared memory in one second!");
        usleep(1000000);
        delete segment;
        ROS_ID_WARN_STREAM("Disconnected from shared memory!");
        return;
      }
      usleep(1000000);
      boost::this_thread::interruption_point();
    }
  }

  bool SharedMemoryTransport::initialized()
  {
    return m_initialized;
  }

  void SharedMemoryTransport::configure(std::string interface_name, std::string field_name, bool create_field)
  {
    PRINT_TRACE_ENTER
    if(m_initialized)
    {
      ROS_ID_WARN_STREAM("Configuring a shared memory transport that had already been configured!");
    }
    else
    {
      ROS_ID_INFO_STREAM("Configuring " << interface_name << ":" << field_name << " transport.");
    }
    while(ros::ok()) //there's probably a much less silly way to do this...
    {
      try
      {
        segment = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());
        break;
      }
      catch(boost::interprocess::interprocess_exception &ex) //shared memory hasn't been created yet, so we'll make it
      {
        ROS_ID_INFO_THROTTLED_STREAM("Waiting for shared memory space " << interface_name << " to become available (is the manager running?)...");
      }
      boost::this_thread::interruption_point();
    }

    m_field_name = field_name;
    m_even_buffer_name = m_field_name + "_even";
    m_odd_buffer_name = m_field_name + "_odd";
    m_buffer_sequence_id_name = m_field_name + "_buffer_sequence_id";
    m_condition_name = m_field_name + "_condition";
    m_condition_mutex_name = m_field_name + "_condition_mutex";
    m_invalid_flag_name = m_field_name + "_invalid";
    m_exists_flag_name = m_field_name + "_exists";

    m_string_allocator = new SMCharAllocator(segment->get_segment_manager());
    m_initialized = true;

    if(create_field)
    {
      if(segment->find<bool>(m_exists_flag_name.c_str()).first != NULL) //check to see if someone else created the field
      {
        ROS_ID_WARN_STREAM("Using existing shared memory field for " << m_field_name);
      }
      else
      {
        createField();
      }
    }
    else
    {
      while(segment->find<bool>(m_exists_flag_name.c_str()).first == NULL)
      {
        ROS_ID_WARN_THROTTLED_STREAM("Waiting for field \"" << m_field_name << "\" to exist");
      }
    }

    m_buffer_sequence_id_ptr = segment->find<uint32_t>(m_buffer_sequence_id_name.c_str()).first;
    m_invalid_ptr = segment->find<bool>(m_invalid_flag_name.c_str()).first;
    m_even_data_ptr = segment->find<SMString>(m_even_buffer_name.c_str()).first;
    m_odd_data_ptr = segment->find<SMString>(m_odd_buffer_name.c_str()).first;
    m_condition_ptr = segment->find<boost::interprocess::interprocess_condition>(m_condition_name.c_str()).first;
    m_condition_mutex_ptr = segment->find<boost::interprocess::interprocess_mutex>(m_condition_mutex_name.c_str()).first;
    m_last_read_buffer_sequence_id = (*m_buffer_sequence_id_ptr) - 1;

    m_watchdog_thread = new boost::thread(boost::bind(&SharedMemoryTransport::watchdogFunction, this));

    m_connected = true;

    ROS_ID_INFO_STREAM("Connected to " << interface_name << ":" << field_name << ".");
    PRINT_TRACE_EXIT
  }

  bool SharedMemoryTransport::createField()
  {
    PRINT_TRACE_ENTER
    TEST_INITIALIZED

    try
    {
      ROS_ID_INFO_STREAM("Creating new shared memory field for " << m_field_name);
      segment->construct<SMString>(m_even_buffer_name.c_str())(*m_string_allocator);
      segment->construct<SMString>(m_odd_buffer_name.c_str())(*m_string_allocator);
      segment->construct<uint32_t>(m_buffer_sequence_id_name.c_str())(0);

      segment->construct<boost::interprocess::interprocess_condition>(m_condition_name.c_str())(); //(segment->get_segment_manager());
      segment->construct<boost::interprocess::interprocess_mutex>(m_condition_mutex_name.c_str())(); //(segment->get_segment_manager());

      segment->construct<bool>(m_invalid_flag_name.c_str())(true); //field is invalid until someone writes actual data to it
      segment->construct<bool>(m_exists_flag_name.c_str())(true); //once we construct this, everyone will assume the field exists
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      ROS_ID_INFO_STREAM("Exception " << ex.what() << " thrown while creating new field \"" << m_field_name << "\"!");
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  inline bool isEven(uint32_t sequence_id)
  {
    return sequence_id & 0x1;
  }

  bool SharedMemoryTransport::getData(std::string& data)
  {
    PRINT_TRACE_ENTER
    if(!hasData())
    {
      PRINT_TRACE_EXIT
      return false;
    }

    int starvation_counter = 0;
    while(ros::ok())
    {
      uint32_t buffer_sequence_id = *m_buffer_sequence_id_ptr;
      SMString* data_ptr = isEven(buffer_sequence_id)? m_even_data_ptr : m_odd_data_ptr;
      try
      {
        data = std::string(data_ptr->begin(), data_ptr->end());
        if(buffer_sequence_id == *m_buffer_sequence_id_ptr) //no one wrote to the buffer while we were trying to read it
        {
          m_last_read_buffer_sequence_id = buffer_sequence_id;
          if(starvation_counter > 2)
          {
            std::cerr << starvation_counter << " starvations" << std::endl;
          }

          PRINT_TRACE_EXIT
          return true;
        }
        else
        {
          starvation_counter++;
        }
      }
      catch(std::exception& ex) //catch std::string issues that happen during the copy
      {
        std::cerr << ex.what() << std::endl;
      }

      boost::this_thread::interruption_point();
    }

    PRINT_TRACE_EXIT
    return false;
  }

  bool SharedMemoryTransport::setData(std::string data)
  {
    PRINT_TRACE_ENTER
    TEST_CONNECTED

    SMString* data_ptr = isEven(*m_buffer_sequence_id_ptr)? m_odd_data_ptr : m_even_data_ptr;
    *data_ptr = SMString(data.begin(), data.end(), *m_string_allocator);
    *m_invalid_ptr = false;
    *m_buffer_sequence_id_ptr = (*m_buffer_sequence_id_ptr) + 1;
    m_condition_ptr->notify_all();

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::hasData()
  {
    PRINT_TRACE_ENTER
    TEST_CONNECTED
    bool has_data = !(*m_invalid_ptr);
    PRINT_TRACE_EXIT
    return has_data;
  }

  bool SharedMemoryTransport::awaitNewDataPolled(std::string& data, double timeout)
  {
    PRINT_TRACE_ENTER
    TEST_CONNECTED

    if(timeout == 0)
    {
      PRINT_TRACE_EXIT
      return getData(data);
    }
    else
    {
      while(ros::ok() && (*m_invalid_ptr)) //wait for the field to at least have something
      {
        CATCH_SHUTDOWN_SIGNAL
        ROS_ID_WARN_THROTTLED_STREAM("Waiting for field " << m_field_name << " to become valid.");
        boost::this_thread::interruption_point();
      }

      boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
      while(ros::ok() && (m_last_read_buffer_sequence_id == *m_buffer_sequence_id_ptr)) //wait for the selector to change
      {
        CATCH_SHUTDOWN_SIGNAL
        if(timeout < 0)
        {
          ROS_ID_DEBUG_THROTTLED_STREAM("Waiting for new data in field " << m_field_name);
        }
        else if(boost::get_system_time() < timeout_time)
        {
          ROS_ID_DEBUG_THROTTLED_STREAM("Waiting for new data in field " << m_field_name << " with timeout " << timeout);
        }
        else
        {
          ROS_ID_INFO_STREAM("Timed out while waiting for new data in field " << m_field_name << " with timeout " << timeout << "!");
          PRINT_TRACE_EXIT
          return false;
        }
        boost::this_thread::interruption_point();
      }

      PRINT_TRACE_EXIT
      return getData(data);
    }
  }

  bool SharedMemoryTransport::awaitNewData(std::string& data, double timeout)
  {
    PRINT_TRACE_ENTER
    TEST_CONNECTED

    if(timeout == 0)
    {
      PRINT_TRACE_EXIT
      return getData(data);
    }
    else if(timeout < 0)
    {
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(*m_condition_mutex_ptr);
      m_condition_ptr->wait(lock);
      lock.unlock();
      return getData(data);
    }
    else
    {
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(*m_condition_mutex_ptr);
      boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
      if(!m_condition_ptr->timed_wait(lock, timeout_time))
      {
        PRINT_TRACE_EXIT
        return false;
      }
      lock.unlock();
      return getData(data);
    }
  }

  std::string SharedMemoryTransport::getFieldName()
  {
    return m_field_name;
  }
}
