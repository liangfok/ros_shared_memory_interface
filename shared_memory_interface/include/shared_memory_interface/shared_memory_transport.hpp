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

#include <vector>
#include <stdio.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_upgradable_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/upgradable_lock.hpp>
#include <boost/thread/locks.hpp>

#include <boost/interprocess/detail/atomic.hpp>

#include <shared_memory_interface/named_upgradable_condition.hpp>

#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <boost/interprocess/exceptions.hpp>
#include <boost/thread/thread_time.hpp>

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

#include "shared_memory_utils.h"

#include <unistd.h>
#include <pwd.h>

namespace shared_memory_interface
{
  typedef boost::interprocess::interprocess_upgradable_mutex upgradable_mutex_type;
  class SharedMemoryTransport
  {
  public:

    SharedMemoryTransport(std::string interface_name);
    ~SharedMemoryTransport();

    static void waitForMemory(std::string interface_name);
    static void createMemory(std::string interface_name);
    static void destroyMemory(std::string interface_name);

    static bool addSerializedField(std::string interface_name, std::string field_name);
    static bool getSerializedField(std::string interface_name, std::string field_name, std::string& data);
    static bool setSerializedField(std::string interface_name, std::string field_name, std::string data);

    static bool exists(std::string interface_name, std::string field_name);
    static bool hasData(std::string interface_name, std::string field_name); //returns true if the field has already been configured
    static bool awaitNewData(std::string interface_name, std::string field_name, double timeout = -1);

  private:
  };

}
#endif //SHARED_MEMORY_TRANSPORT_HPP
