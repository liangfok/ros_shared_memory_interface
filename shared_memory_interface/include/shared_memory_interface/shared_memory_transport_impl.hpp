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

#ifndef SHARED_MEMORY_TRANSPORT_IMPL_HPP
#define SHARED_MEMORY_TRANSPORT_IMPL_HPP

#include "shared_memory_interface/shared_memory_transport.hpp"
#include "shared_memory_utils.hpp"

namespace shared_memory_interface
{
  template<typename T>
  SharedMemoryTransport<T>::SharedMemoryTransport(unsigned long reservation_size)
  {
    m_reservation_size = reservation_size;
    m_initialized = false;
    m_connected = false;
    m_watchdog_thread = NULL;
    m_already_read_valid = false;
    m_already_set_valid = false;
  }

  template<typename T>
  SharedMemoryTransport<T>::~SharedMemoryTransport()
  {
    if(m_watchdog_thread != NULL)
    {
      m_watchdog_thread->interrupt();
      m_watchdog_thread->detach();
      delete m_watchdog_thread;
    }
  }

  template<typename T>
  void SharedMemoryTransport<T>::watchdogFunction()
  {
    bool* shutdown_required_ptr = NULL;
    ros::Rate loop_rate(2.0);
    while(ros::ok())
    {
      shutdown_required_ptr = segment->find<bool>("shutdown_required").first;
      if(shutdown_required_ptr)
      {
        break;
      }
      ROS_ID_WARN_THROTTLED_STREAM("Watchdog waiting for shutdown signal field...");
      loop_rate.sleep();
    }

    while(ros::ok())
    {
      if(*shutdown_required_ptr)
      {
        m_initialized = false;
        ROS_ID_WARN_STREAM("Shutdown signal detected! Disconnecting from shared memory in one second!");
        usleep(1000000);
        delete segment;
        ROS_ID_WARN_STREAM("Disconnected from shared memory!");
        return;
      }
      loop_rate.sleep();
    }
  }

  template<typename T>
  bool SharedMemoryTransport<T>::initialized()
  {
    return m_initialized;
  }

  template<typename T>
  bool SharedMemoryTransport<T>::connected()
  {
    return m_connected;
  }

  template<typename T>
  void SharedMemoryTransport<T>::configure(std::string interface_name, std::string field_name, bool create_field)
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
      //boost::this_thread::interruption_point();
    }

    m_field_name = field_name;
    m_interface_name = interface_name;
    m_even_buffer_name = m_field_name + "_e";
    m_even_length_name = m_field_name + "_el";
    m_odd_buffer_name = m_field_name + "_o";
    m_odd_length_name = m_field_name + "_ol";
    m_buffer_sequence_id_name = m_field_name + "_b";
    m_condition_name = m_field_name + "_c";
    m_condition_mutex_name = m_field_name + "_cm";
    m_invalid_flag_name = m_field_name + "_i";
    m_exists_flag_name = m_field_name + "_ex";

    m_string_allocator = new SMCharAllocator(segment->get_segment_manager());

    m_watchdog_thread = new boost::thread(boost::bind(&SharedMemoryTransport::watchdogFunction, this));

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

    ROS_ID_INFO_STREAM("Configured transport for " << m_interface_name << ":" << m_field_name << ".");
    PRINT_TRACE_EXIT
  }

  template<typename T>
  bool SharedMemoryTransport<T>::connect(double timeout)
  {
    if(!m_initialized)
    {
      ROS_ID_ERROR_STREAM("Connect called on uninitialized transport");
      return false;
    }

    ROS_ID_DEBUG_THROTTLED_STREAM("Attempting to connect to " << m_interface_name << ":" << m_field_name << ".");
    if(timeout == 0.0 && (segment->find<bool>(m_exists_flag_name.c_str()).first == NULL))
    {
      ROS_ID_DEBUG_THROTTLED_STREAM("Failed while attempting to connect to " << m_interface_name << ":" << m_field_name << " with immediate timeout!");
      return false;
    }
    else if(timeout < 0.0)
    {
      while(segment->find<bool>(m_exists_flag_name.c_str()).first == NULL)
      {
        ROS_ID_WARN_THROTTLED_STREAM("Waiting for field \"" << m_field_name << "\" to exist");
      }
    }
    else
    {
      boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
      while(segment->find<bool>(m_exists_flag_name.c_str()).first == NULL)
      {
        ROS_ID_WARN_THROTTLED_STREAM("Waiting for field \"" << m_field_name << "\" to exist");
        if(boost::get_system_time() >= timeout_time)
        {
          ROS_ID_WARN_THROTTLED_STREAM("Failed while attempting to connect to " << m_interface_name << ":" << m_field_name << " with timeout " << timeout << "!");
          return false;
        }
      }
    }

    m_buffer_sequence_id_ptr = segment->find<uint32_t>(m_buffer_sequence_id_name.c_str()).first;
    m_invalid_ptr = segment->find<bool>(m_invalid_flag_name.c_str()).first;
    m_even_string_ptr = segment->find<SMString>(m_even_buffer_name.c_str()).first;
    m_even_data_ptr = (unsigned char*) &(m_even_string_ptr->at(0));
    m_even_length_ptr = segment->find<uint32_t>(m_even_length_name.c_str()).first;
    m_odd_string_ptr = segment->find<SMString>(m_odd_buffer_name.c_str()).first;
    m_odd_data_ptr = (unsigned char*) &(m_odd_string_ptr->at(0));
    m_odd_length_ptr = segment->find<uint32_t>(m_odd_length_name.c_str()).first;
    m_condition_ptr = segment->find<boost::interprocess::interprocess_condition>(m_condition_name.c_str()).first;
    m_condition_mutex_ptr = segment->find<boost::interprocess::interprocess_mutex>(m_condition_mutex_name.c_str()).first;
    m_last_read_buffer_sequence_id = (*m_buffer_sequence_id_ptr) - 1;

    m_connected = true;

    ROS_ID_INFO_STREAM("Connected to " << m_interface_name << ":" << m_field_name << ".");

    return true;
  }

  template<typename T>
  bool SharedMemoryTransport<T>::createField()
  {
    PRINT_TRACE_ENTER
    TEST_INITIALIZED

    try
    {
      ROS_ID_INFO_STREAM("Creating new shared memory field for " << m_field_name);
      segment->construct<SMString>(m_even_buffer_name.c_str())(*m_string_allocator);
      segment->construct<uint32_t>(m_even_length_name.c_str())(m_reservation_size);
      segment->construct<SMString>(m_odd_buffer_name.c_str())(*m_string_allocator);
      segment->construct<uint32_t>(m_odd_length_name.c_str())(m_reservation_size);
      segment->construct<uint32_t>(m_buffer_sequence_id_name.c_str())(0);

      segment->find<SMString>(m_even_buffer_name.c_str()).first->resize(m_reservation_size);
      segment->find<SMString>(m_odd_buffer_name.c_str()).first->resize(m_reservation_size);

      segment->construct<boost::interprocess::interprocess_condition>(m_condition_name.c_str())();
      segment->construct<boost::interprocess::interprocess_mutex>(m_condition_mutex_name.c_str())();

      segment->construct<bool>(m_invalid_flag_name.c_str())(true); //field is invalid until someone writes actual data to it
      segment->construct<bool>(m_exists_flag_name.c_str())(true); //once we construct this, everyone will assume the field exists
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      ROS_ID_ERROR_STREAM("\n\n\n\n=======================================================================================================");
      ROS_ID_ERROR_STREAM("=======================================================================================================");
      ROS_ID_ERROR_STREAM("CRITICAL ERROR! EXCEPTION " << ex.what() << " THROWN WHILE CREATING \"" << m_field_name << "\"! THIS SHOULD NEVER HAPPEN!");
      ROS_ID_ERROR_STREAM("=======================================================================================================");
      ROS_ID_ERROR_STREAM("=======================================================================================================\n\n\n\n");

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

  template<typename T>
  bool SharedMemoryTransport<T>::getData(T& data)
  {
    PRINT_TRACE_ENTER
    if(!m_already_read_valid && !hasData())
    {
      PRINT_TRACE_EXIT
      return false;
    }

    int starvation_counter = 0;
    while(ros::ok())
    {
      uint32_t buffer_sequence_id = *m_buffer_sequence_id_ptr;
      bool even = isEven(buffer_sequence_id);
      unsigned char* data_ptr = even? m_even_data_ptr : m_odd_data_ptr;
      uint32_t* length_ptr = even? m_even_length_ptr : m_odd_length_ptr;
      try
      {
        ros::serialization::IStream istream(data_ptr, *length_ptr);
        ros::serialization::deserialize(istream, data);
        if(buffer_sequence_id == *m_buffer_sequence_id_ptr) //no one wrote to the buffer while we were trying to read it
        {
          m_last_read_buffer_sequence_id = buffer_sequence_id;
          if(starvation_counter > 2)
          {
            ROS_ID_WARN_STREAM(starvation_counter << " starvations while getting data from field " << m_field_name);
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
        std::cerr << "Exception " << ex.what() << " occurred while getting data from field " << m_field_name << std::endl;
      }

      //boost::this_thread::interruption_point();
    }

    PRINT_TRACE_EXIT
    return false;
  }

  template<typename T>
  bool SharedMemoryTransport<T>::setData(T& data)
  {
    PRINT_TRACE_ENTER
    TEST_CONNECTED

    unsigned long oserial_size = ros::serialization::serializationLength(data);

    //todo make sure we resize if we don't fit!

    unsigned char* data_ptr;
    uint32_t* length_ptr;
    uint32_t buffer_sequence_id = *m_buffer_sequence_id_ptr;
    if(isEven(buffer_sequence_id))
    {
      data_ptr = m_odd_data_ptr;
      length_ptr = m_odd_length_ptr;
    }
    else
    {
      data_ptr = m_even_data_ptr;
      length_ptr = m_even_length_ptr;
    }
    ros::serialization::OStream ostream(data_ptr, oserial_size);
    ros::serialization::serialize(ostream, data);

    *length_ptr = oserial_size;

//    if(!m_already_set_valid)
//    {
    *m_invalid_ptr = false;
//      m_already_set_valid = true;
//    }
    *m_buffer_sequence_id_ptr = buffer_sequence_id + 1;
    m_condition_ptr->notify_all();

    PRINT_TRACE_EXIT
    return true;
  }

  template<typename T>
  bool SharedMemoryTransport<T>::hasData()
  {
    PRINT_TRACE_ENTER
    bool has_data = !(*m_invalid_ptr);
    m_already_read_valid = has_data;
    PRINT_TRACE_EXIT
    return has_data;
  }

  template<typename T>
  bool SharedMemoryTransport<T>::awaitNewDataPolled(T& data, double timeout)
  {
    PRINT_TRACE_ENTER
    TEST_CONNECTED

    if(timeout == 0)
    {
      PRINT_TRACE_EXIT
      return getData(data);
    }

    if(!m_already_read_valid)
    {
      while(ros::ok() && (*m_invalid_ptr)) //wait for the field to at least have something
      {
        CATCH_SHUTDOWN_SIGNAL
        ROS_ID_WARN_THROTTLED_STREAM("Waiting for field " << m_field_name << " to become valid.");
        //boost::this_thread::interruption_point();
      }
      m_already_read_valid = true;
    }

    if(timeout < 0)
    {
      while(ros::ok() && (m_last_read_buffer_sequence_id == *m_buffer_sequence_id_ptr)) //wait for the selector to change
      {
        CATCH_SHUTDOWN_SIGNAL
      }
    }
    else
    {
      boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
      while(ros::ok() && (m_last_read_buffer_sequence_id == *m_buffer_sequence_id_ptr)) //wait for the selector to change
      {
        CATCH_SHUTDOWN_SIGNAL
        if(timeout < 0)
        {
//          ROS_ID_DEBUG_THROTTLED_STREAM("Waiting for new data in field " << m_field_name);
        }
        else if(boost::get_system_time() < timeout_time)
        {
//          ROS_ID_DEBUG_THROTTLED_STREAM("Waiting for new data in field " << m_field_name << " with timeout " << timeout);
        }
        else
        {
          ROS_ID_INFO_STREAM("Timed out while waiting for new data in field " << m_field_name << " with timeout " << timeout << "!");
          PRINT_TRACE_EXIT
          return false;
        }
        //boost::this_thread::interruption_point();
      }
    }

    PRINT_TRACE_EXIT
    return getData(data);
  }

  template<typename T>
  bool SharedMemoryTransport<T>::awaitNewData(T& data, double timeout)
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

  template<typename T>
  std::string SharedMemoryTransport<T>::getFieldName()
  {
    return m_field_name;
  }
}
#endif //SHARED_MEMORY_TRANSPORT_IMPL_HPP
