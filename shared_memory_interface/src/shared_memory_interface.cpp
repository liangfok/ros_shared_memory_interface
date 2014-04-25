#include "shared_memory_interface/shared_memory_interface.hpp"

namespace shared_memory_interface
{
  SharedMemoryInterface::SharedMemoryInterface(std::string interface_name) :
      m_smt(interface_name)
  {
    m_interface_name = interface_name;
  }

  SharedMemoryInterface::~SharedMemoryInterface()
  {
  }

  void SharedMemoryInterface::destroyMemory(std::string interface_name)
  {
    SharedMemoryTransport::destroyMemory(interface_name);
  }

  bool SharedMemoryInterface::advertiseStringVector(std::string field_name, unsigned long length)
  {
    return true;
  }

  bool SharedMemoryInterface::advertiseFPVector(std::string field_name, unsigned long length)
  {
    return true;
  }

  bool SharedMemoryInterface::advertiseFPMatrix(std::string field_name, unsigned long rows, unsigned long cols)
  {
    return true;
  }

  bool SharedMemoryInterface::publishStringVector(std::string field, std::vector<std::string>& data)
  {
    return true;
  }

  bool SharedMemoryInterface::publishFPVector(std::string field, std::vector<double>& data)
  {
    return true;
  }

  bool SharedMemoryInterface::publishFPMatrix(std::string field, std::vector<double>& data)
  {
    return true;
  }

  bool SharedMemoryInterface::subscribeStringVector(std::string field, boost::function<void(std::vector<double>&)> callback)
  {
    return true;
  }

  bool SharedMemoryInterface::subscribeFPVector(std::string field, boost::function<void(std::vector<double>&)> callback)
  {
    return true;
  }

  bool SharedMemoryInterface::subscribeFPMatrix(std::string field, boost::function<void(std::vector<double>&)> callback)
  {
    return true;
  }
}
