#ifndef SHARED_MEMORY_TRANSPORT_HPP
#define SHARED_MEMORY_TRANSPORT_HPP

#include <vector>
#include <stdio.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <boost/interprocess/exceptions.hpp>
#include <boost/thread/thread_time.hpp>

namespace shared_memory_interface
{
  class SharedMemoryTransport
  {
  public:

    SharedMemoryTransport(std::string interface_name);
    ~SharedMemoryTransport();

    static void destroyMemory(std::string interface_name);

    bool addMatrixField(std::string field_name, unsigned long rows, unsigned long cols, std::string sm_namespace = "");

    bool addJointField(std::string field_name, unsigned long num_joints, std::string sm_namespace = "");

    bool getData(std::string field, unsigned long joint_idx, double& data, std::string sm_namespace = "");

    bool setData(std::string field, unsigned long joint_idx, double value, std::string sm_namespace = "");

    bool getField(std::string field, std::vector<double>& field_data_local, std::string sm_namespace = "");

    bool setField(std::string field, std::vector<double>& field_data_local, std::string sm_namespace = "");

    bool getJointNames(std::vector<std::string>& names_local, std::string sm_namespace = "");

    bool setJointNames(std::vector<std::string> names_local, std::string sm_namespace = "");

    bool setTxSequenceNumber(unsigned char value);

    bool getTxSequenceNumber(unsigned char& sequence_number);

    bool setRxSequenceNumber(unsigned char value);

    bool getRxSequenceNumber(unsigned char& sequence_number);

    bool hasConnections();

    bool hasNew(std::string field_name, std::string sm_namespace = "");

    bool signalAvailable(std::string field_name, std::string sm_namespace = "");

    bool signalProcessed(std::string field_name, std::string sm_namespace = "");

  private:
    boost::interprocess::named_mutex* m_mutex;
    std::string m_interface_name;
    std::string m_mutex_name;
    std::string m_data_name;
  };

}
#endif //SHARED_MEMORY_TRANSPORT_HPP
