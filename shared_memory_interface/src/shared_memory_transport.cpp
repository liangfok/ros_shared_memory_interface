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

  SharedMemoryTransport::SharedMemoryTransport(std::string interface_name)
  {
  }

  SharedMemoryTransport::~SharedMemoryTransport()
  {
  }

  void SharedMemoryTransport::waitForMemory(std::string interface_name)
  {
    while(true) //there's probably a much less silly way to do this...
    {
      try
      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());
        std::cerr << "Connected to " << interface_name << " space." << std::endl;
        break;
      }
      catch(boost::interprocess::interprocess_exception &ex) //shared memory hasn't been created yet, so we'll make it
      {
        std::cerr << "Waiting for shared memory space " << interface_name << " to become available (is the manager running?)..." << std::endl;
      }
    }
  }

  void SharedMemoryTransport::createMemory(std::string interface_name, unsigned int size)
  {
    try
    {
      std::cerr << "Creating shared memory space " << interface_name << ".." << std::endl;
      unsigned long base_size = 8192; //make sure we have enough room to create the default objects
      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::create_only, interface_name.c_str(), base_size, NULL, unrestricted());
      }
      boost::interprocess::managed_shared_memory::shrink_to_fit(interface_name.c_str());
      std::cerr << "Created " << interface_name << " space!" << std::endl;
    }
    catch(boost::interprocess::interprocess_exception &ex) //shared memory hasn't been created yet, so we'll make it
    {
      std::cerr << "ERROR: shared memory space already existed!";
    }
  }

  void SharedMemoryTransport::destroyMemory(std::string interface_name)
  {
    std::cerr << "Destroying shared memory space " << interface_name << "." << std::endl;
    boost::interprocess::shared_memory_object::remove(interface_name.c_str());
    SMScopedLock::destroy(interface_name, "");
  }

  void expandMemory(std::string interface_name, unsigned int num_bytes)
  {
    SMScopedWriterLock memory_lock(interface_name, "");
    boost::interprocess::managed_shared_memory::grow(interface_name.c_str(), num_bytes);
  }

#define DEFAULT_FIELD_SIZE 1000000
#define OVERHEAD_FACTOR 10 //overestimate that it takes 10x more bytes than requested
  bool SharedMemoryTransport::addSerializedField(std::string interface_name, std::string field_name)
  {
    PRINT_TRACE_ENTER

    //check to see if someone else created the field
    if(exists(interface_name, field_name))
    {
      PRINT_TRACE_EXIT
      return true;
    }

    expandMemory(interface_name, 2 * DEFAULT_FIELD_SIZE);

    try
    {
      SMScopedWriterLock memory_lock(interface_name, "");
      std::cerr << "Creating new shared memory field for " << field_name << std::endl;
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());

      SMString* string_sm_0 = segment.construct<SMString>((field_name + "_0").c_str())(segment.get_segment_manager());
      SMString* string_sm_1 = segment.construct<SMString>((field_name + "_1").c_str())(segment.get_segment_manager());

      //reserve space now to reduce computation later
      string_sm_0->reserve(DEFAULT_FIELD_SIZE / OVERHEAD_FACTOR);
      string_sm_1->reserve(DEFAULT_FIELD_SIZE / OVERHEAD_FACTOR);

      segment.construct<bool>(std::string(field_name + "_latest_data_in_zero").c_str())(false);
      segment.construct<bool>(std::string(field_name + "_invalid").c_str())(true); //field is invalid until someone writes actual data to it

      segment.construct<boost::interprocess::interprocess_condition>(std::string(field_name + "_condition").c_str())(segment.get_segment_manager());
      segment.construct<boost::interprocess::interprocess_mutex>(std::string(field_name + "_condition_mutex").c_str())(segment.get_segment_manager());
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while creating new field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getSerializedField(std::string interface_name, std::string field_name, std::string& data)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(interface_name, "");
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());

    bool success = false;
    while(!success)
    {
      bool latest_data_in_zero = *segment.find<bool>(std::string(field_name + "_latest_data_in_zero").c_str()).first;
      std::string read_buffer_name;
      if(latest_data_in_zero)
      {
        read_buffer_name = (field_name + "_0");
      }
      else
      {
        read_buffer_name = (field_name + "_1");
      }

      SMString* field_data = segment.find<SMString>(read_buffer_name.c_str()).first;
      if(field_data == NULL) //field_name doesn't exist yet
      {
        std::cerr << "Couldn't find field_name " << field_name << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }

      if(latest_data_in_zero != *segment.find<bool>(std::string(field_name + "_latest_data_in_zero").c_str()).first)
      {
        //someone wrote to the buffer while we were reading it! try again...
        continue;
      }

      data = std::string(field_data->begin(), field_data->end());

      success = true;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::setSerializedField(std::string interface_name, std::string field_name, std::string data)
  {
    PRINT_TRACE_ENTER

    unsigned long capacity;
    std::string write_buffer_name;
    bool latest_data_in_zero;

    {
      SMScopedReaderLock memory_lock(interface_name, "");
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());
      latest_data_in_zero = *segment.find<bool>(std::string(field_name + "_latest_data_in_zero").c_str()).first;
      if(latest_data_in_zero)
      {
        write_buffer_name = (field_name + "_1");
      }
      else
      {
        write_buffer_name = (field_name + "_0");
      }

      SMString* field_data = segment.find<SMString>(write_buffer_name.c_str()).first;
      if(field_data == NULL) //field_name doesn't exist yet
      {
        std::cerr << "Couldn't find field_name " << field_name << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }

      capacity = field_data->capacity();
      if(capacity > data.length())
      {
        *field_data = SMString(data.begin(), data.end(), segment.get_segment_manager());
        *segment.find<bool>((field_name + "_invalid").c_str()).first = false;
        *segment.find<bool>((field_name + "_latest_data_in_zero").c_str()).first = !latest_data_in_zero;
        segment.find<boost::interprocess::interprocess_condition>((field_name + "_condition").c_str()).first->notify_all();
      }
    }

    if(capacity <= data.length()) //need to resize to have enough room
    {
      expandMemory(interface_name, 2 * data.length() * OVERHEAD_FACTOR);
      SMScopedReaderLock memory_lock(interface_name, "");
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());
      *segment.find<SMString>(write_buffer_name.c_str()).first = SMString(data.begin(), data.end(), segment.get_segment_manager());
      *segment.find<bool>((field_name + "_invalid").c_str()).first = false;
      *segment.find<bool>((field_name + "_latest_data_in_zero").c_str()).first = !latest_data_in_zero;
      segment.find<boost::interprocess::interprocess_condition>((field_name + "_condition").c_str()).first->notify_all();
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::exists(std::string interface_name, std::string field_name)
  {
    SMScopedReaderLock memory_lock(interface_name, "");
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());

    if((segment.find<SMString>(std::string(field_name + "_0").c_str()).first == 0) || (segment.find<SMString>(std::string(field_name + "_1").c_str()).first == 0)) //field_name doesn't exist
    {
      PRINT_TRACE_EXIT
      return false;
    }
    return true;
  }

  bool SharedMemoryTransport::hasData(std::string interface_name, std::string field_name)
  {
    PRINT_TRACE_ENTER

    if(!exists(interface_name, field_name))
    {
      return false;
    }

    SMScopedReaderLock memory_lock(interface_name, "");
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());

    bool* invalid = segment.find<bool>((field_name + "_invalid").c_str()).first;
    if(invalid == NULL)
    {
      std::cerr << "Companion fields for field " << field_name << " don't exist!" << std::endl;
      return false;
    }
    if(*invalid)
    {
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::awaitNewData(std::string interface_name, std::string field_name, double timeout)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::named_upgradable_mutex* memory_mutex = new boost::interprocess::named_upgradable_mutex(boost::interprocess::open_only, SMScopedLock::getFullName(interface_name, field_name).c_str());
    memory_mutex->lock();

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, interface_name.c_str());
    segment.find<boost::interprocess::interprocess_condition>((field_name + "_condition").c_str()).first->wait(memory_mutex);

    if(timeout < 0)
    {
      boost::interprocess::named_condition(boost::interprocess::open_or_create, (field_name + "_ready").c_str(), unrestricted()).wait(memory_mutex);
    }
    else
    {
      boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
      if(!boost::interprocess::named_condition(boost::interprocess::open_or_create, (field_name + "_ready").c_str(), unrestricted()).timed_wait(lock, timeout_time))
      {
        PRINT_TRACE_EXIT
        return false;
      }
    }
    PRINT_TRACE_EXIT
    return true;
  }

//  bool SharedMemoryTransport::signalAvailable(std::string interface_name, std::string field_name)
//  {
//    PRINT_TRACE_ENTER
//    SMScopedWriterLock lock(m_interface_name, field_name);
//
//    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
//    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;
//    if(!flag)
//    {
//      PRINT_TRACE_EXIT
//      return false;
//    }
//
//    boost::interprocess::named_condition(boost::interprocess::open_or_create, (field_name + "_ready").c_str(), unrestricted()).notify_all();
//
//    *flag = true;
//    PRINT_TRACE_EXIT
//    return true;
//  }
//
//  bool SharedMemoryTransport::signalProcessed(std::string interface_name, std::string field_name)
//  {
//    PRINT_TRACE_ENTER
//    SMScopedWriterLock lock(m_interface_name, field_name);
//
//    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
//    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;
//    if(!flag)
//    {
//      PRINT_TRACE_EXIT
//      return false;
//    }
//
//    *flag = false;
//    PRINT_TRACE_EXIT
//    return true;
//  }
}
