#ifndef SHARED_MEMORY_INTERFACE_HPP
#define SHARED_MEMORY_INTERFACE_HPP

#include "shared_memory_interface/shared_memory_transport.hpp"

#include <vector>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace shared_memory_interface
{
  class SharedMemoryInterface
  {
  public:
    SharedMemoryInterface(std::string interface_name);
    ~SharedMemoryInterface();

    static void destroyMemory(std::string interface_name);

    //high efficiency implementations, limited data types
    bool advertiseStringVector(std::string field_name, unsigned long length);
    bool advertiseFPVector(std::string field_name, unsigned long length);
    bool advertiseFPMatrix(std::string field_name, unsigned long rows, unsigned long cols);

    bool publishStringVector(std::string field, std::vector<std::string>& data);
    bool publishFPVector(std::string field, std::vector<double>& data);
    bool publishFPMatrix(std::string field, std::vector<double>& data); //TODO: add implementation that takes an actual matrix type

    bool subscribeStringVector(std::string field, boost::function<void(std::vector<double>&)> callback);
    bool subscribeFPVector(std::string field, boost::function<void(std::vector<double>&)> callback);
    bool subscribeFPMatrix(std::string field, boost::function<void(std::vector<double>&)> callback);

    //generic implementations, works with any ROS message type
    //TODO: work templated serialization magics
//    bool advertiseMessage<>(std::string field_name);


  private:
    SharedMemoryTransport m_smt;
    std::string m_interface_name;
  };

}
#endif //SHARED_MEMORY_INTERFACE_HPP
