#include "shared_memory_interface/shared_memory_interface.hpp"

namespace shared_memory_interface
{
  SharedMemoryInterface::SharedMemoryInterface(std::string interface_name) :
      m_smt(interface_name)
  {
    m_interface_name = interface_name;
    m_callback_thread = new boost::thread(boost::bind(&SharedMemoryInterface::callbackThreadFunction, this));
    m_shutdown = false;
  }

  SharedMemoryInterface::~SharedMemoryInterface()
  {
    m_shutdown = true;
    //TODO: delete m_callback_thread?
  }

  void SharedMemoryInterface::destroyMemory(std::string interface_name)
  {
    SharedMemoryTransport::destroyMemory(interface_name);
  }

  //high efficiency implementations, limited data types
  bool SharedMemoryInterface::advertiseStringVector(std::string field_name, unsigned long length)
  {
    return m_smt.addSVField(field_name, length);
  }

  bool SharedMemoryInterface::advertiseFPVector(std::string field_name, unsigned long length)
  {
    return m_smt.addFPMatrixField(field_name, 1, length);
  }

  bool SharedMemoryInterface::advertiseFPMatrix(std::string field_name, unsigned long rows, unsigned long cols)
  {
    return m_smt.addFPMatrixField(field_name, rows, cols);
  }

  bool SharedMemoryInterface::publishStringVector(std::string field_name, std::vector<std::string>& data)
  {
    return m_smt.setSVField(field_name, data);
  }

  bool SharedMemoryInterface::publishFPVector(std::string field_name, std::vector<double>& data)
  {
    return m_smt.setFPField(field_name, data);
  }

  bool SharedMemoryInterface::publishFPMatrix(std::string field_name, std::vector<double>& data)
  {
    return m_smt.setFPField(field_name, data);
  }

  bool SharedMemoryInterface::subscribeStringVector(std::string field_name, boost::function<void(std::vector<std::string>&)> callback)
  {
    m_string_subscriptions[field_name] = callback;
    return true;
  }

  bool SharedMemoryInterface::subscribeFPVector(std::string field_name, boost::function<void(std::vector<double>&)> callback)
  {
    m_FP_subscriptions[field_name] = callback;
    return true;
  }

  bool SharedMemoryInterface::subscribeFPMatrix(std::string field_name, boost::function<void(std::vector<double>&)> callback)
  {
    m_FP_subscriptions[field_name] = callback;
    return true;
  }

  //generic implementations, works with any ROS message type
  //TODO

  //callback
  void SharedMemoryInterface::callbackThreadFunction()
  {
    //TODO: use conditional variables to improve efficiency
    //TODO: add ability to get current value even if not new
    while(!m_shutdown)
    {
      //todo: scoped mutex to prevent callback addition while we're checking
      typedef std::map<std::string, boost::function<void(std::vector<double>&)> >::iterator FPMapIter;
      for(FPMapIter iter = m_FP_subscriptions.begin(); iter != m_FP_subscriptions.end(); iter++)
      {
        if(m_smt.hasNew(iter->first))
        {
          std::vector<double> data;
          m_smt.getFPField(iter->first, data);
          iter->second(data);
        }
      }

      typedef std::map<std::string, boost::function<void(std::vector<std::string>&)> >::iterator StringMapIter;
      for(StringMapIter iter = m_string_subscriptions.begin(); iter != m_string_subscriptions.end(); iter++)
      {
        if(m_smt.hasNew(iter->first))
        {
          //TODO:
        std::vector<std::string> data;
        m_smt.getSVField(iter->first, data);
        iter->second(data);
        }
      }
    }
  }
}
