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

#ifndef SHARED_MEMORY_TRANSPORT_HPP
#define SHARED_MEMORY_TRANSPORT_HPP

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

#include "shared_memory_utils.hpp"

namespace shared_memory_interface
{
  class SharedMemoryTransport
  {
  public:
    SharedMemoryTransport();
    ~SharedMemoryTransport();

    static bool createMemory(std::string interface_name, unsigned int size);
    static void destroyMemory(std::string interface_name);

    bool initialized();
    void configure(std::string interface_name, std::string field_name, bool create_field=false);
    bool createField();
    bool getData(std::string& data);
    bool setData(std::string data);

    std::string getFieldName();

    bool hasData(); //returns true if the field has already been configured
    bool awaitNewDataPolled(std::string& data, double timeout = -1);
    bool awaitNewData(std::string& data, double timeout = -1);

  private:
    boost::interprocess::managed_shared_memory* segment;
    boost::thread* m_watchdog_thread;
    void watchdogFunction();

    bool m_initialized;
    bool m_connected;
    std::string m_field_name;
    std::string m_even_buffer_name;
    std::string m_odd_buffer_name;
    std::string m_buffer_sequence_id_name;
    std::string m_invalid_flag_name;
    std::string m_exists_flag_name;

    uint32_t* m_buffer_sequence_id_ptr;
    bool* m_invalid_ptr;
    SMString* m_even_data_ptr;
    SMString* m_odd_data_ptr;

    uint32_t m_last_read_buffer_sequence_id;
  };

}
#endif //SHARED_MEMORY_TRANSPORT_HPP
