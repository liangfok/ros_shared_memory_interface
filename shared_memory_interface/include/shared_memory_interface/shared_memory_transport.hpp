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

    static void destroyMemory(std::string interface_name);

    //FloatingPoint=> floating point
    //StringVector=> string vector

    bool addFloatingPointMatrixField(std::string field_name, unsigned long rows, unsigned long cols);

    bool addStringVectorField(std::string field_name, unsigned long length);

    bool addSerializedField(std::string field_name, std::string md5sum, std::string datatype);

    bool getFloatingPointData(std::string field_name, unsigned long joint_idx, double& data);

    bool setFloatingPointData(std::string field_name, unsigned long joint_idx, double value);

    //TODO: return row stride as well?
    bool getFloatingPointField(std::string field_name, std::vector<double>& field_data_local);

    bool setFloatingPointField(std::string field_name, std::vector<double>& field_data_local);

    bool checkFloatingPointField(std::string field_name); //returns true if the field has already been configured

    bool getStringVectorField(std::string field_name, std::vector<std::string>& names_local);

    bool setStringVectorField(std::string field_name, std::vector<std::string>& names_local);

    bool checkStringVectorField(std::string field_name); //returns true if the field has already been configured

    bool getSerializedField(std::string field_name, std::string& data, std::string& md5sum, std::string& datatype);

    bool setSerializedField(std::string field_name, std::string data, std::string md5sum, std::string datatype);

    bool checkSerializedField(std::string field_name); //returns true if the field has already been configured

    bool hasConnections();

    bool hasNewData(std::string field_name);

    bool awaitNewData(std::string field_name, double timeout = -1);

    bool signalAvailable(std::string field_name);

    bool signalProcessed(std::string field_name);

  private:
//    boost::interprocess::named_upgradable_mutex* m_mutex;
    std::string m_interface_name;
//    std::string m_memory_mutex_name;
    std::string m_data_name;
  };

}
#endif //SHARED_MEMORY_TRANSPORT_HPP
