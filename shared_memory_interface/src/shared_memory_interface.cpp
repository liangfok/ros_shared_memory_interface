#include "shared_memory_interface/shared_memory_interface.hpp"

namespace shared_memory_interface
{
  SharedMemoryInterface::SharedMemoryInterface(std::string interface_name) :
      m_smt(interface_name)
  {
    m_shutdown = false;
    m_interface_name = interface_name;
  }

  SharedMemoryInterface::~SharedMemoryInterface()
  {
    m_shutdown = true;
  }

  void SharedMemoryInterface::destroyMemory(std::string interface_name)
  {
    SharedMemoryTransport::destroyMemory(interface_name);
  }

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
    if(m_smt.setSVField(field_name, data))
    {
      if(m_smt.signalAvailable(field_name))
      {
        return true;
      }
    }
    return false;
  }

  bool SharedMemoryInterface::publishFPVector(std::string field_name, std::vector<double>& data)
  {
    if(m_smt.setFPField(field_name, data))
    {
      if(m_smt.signalAvailable(field_name))
      {
        return true;
      }
    }
    return false;
  }

  bool SharedMemoryInterface::publishFPMatrix(std::string field_name, std::vector<double>& data)
  {
    if(m_smt.setFPField(field_name, data))
    {
      if(m_smt.signalAvailable(field_name))
      {
        return true;
      }
    }
    return false;
  }

  bool SharedMemoryInterface::subscribeStringVector(std::string field_name, boost::function<void(std::vector<std::string>&)> callback)
  {
    if(!m_smt.checkSVField(field_name)) //if this field hasn't been advertised yet, go ahead and make it
    {
      advertiseStringVector(field_name, 1); //the field needs to be created for the await method to work
    }
    m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterface::callbackThreadFunctionSV, this, field_name, callback)));
    return true;
  }

  bool SharedMemoryInterface::subscribeFPVector(std::string field_name, boost::function<void(std::vector<double>&)> callback)
  {
    if(!m_smt.checkSVField(field_name)) //if this field hasn't been advertised yet, go ahead and make it
    {
      advertiseFPVector(field_name, 1); //the field needs to be created for the await method to work
    }
    m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterface::callbackThreadFunctionFP, this, field_name, callback)));
    return true;
  }

  bool SharedMemoryInterface::subscribeFPMatrix(std::string field_name, boost::function<void(std::vector<double>&)> callback)
  {
    if(!m_smt.checkSVField(field_name)) //if this field hasn't been advertised yet, go ahead and make it
    {
      advertiseFPMatrix(field_name, 1, 1); //the field needs to be created for the await method to work
    }
    m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterface::callbackThreadFunctionFP, this, field_name, callback)));
    return true;
  }

  bool SharedMemoryInterface::waitForStringVector(std::string field_name, std::vector<std::string>& data, double timeout)
  {
    m_smt.awaitNewData(field_name, timeout);
    m_smt.getSVField(field_name, data);
    m_smt.signalProcessed(field_name);
  }

  bool SharedMemoryInterface::waitForFPVector(std::string field_name, std::vector<double>& data, double timeout)
  {
    m_smt.awaitNewData(field_name, timeout);
    m_smt.getFPField(field_name, data);
    m_smt.signalProcessed(field_name);
  }

  bool SharedMemoryInterface::waitForFPMatrix(std::string field_name, std::vector<double>& data, double timeout)
  {
    m_smt.awaitNewData(field_name, timeout);
    m_smt.getFPField(field_name, data);
    m_smt.signalProcessed(field_name);
  }

  //private callback threads
  void SharedMemoryInterface::callbackThreadFunctionFP(std::string field_name, boost::function<void(std::vector<double>&)> callback)
  {
    while(!m_smt.checkFPField(field_name)) //wait for the field to be advertised
    {
      //TODO: add a warning?
      usleep(10000);
    }

    while(!m_shutdown)
    {
      m_smt.awaitNewData(field_name);

      std::vector<double> data;
      m_smt.getFPField(field_name, data);
      callback(data);
      m_smt.signalProcessed(field_name);
    }
  }

  void SharedMemoryInterface::callbackThreadFunctionSV(std::string field_name, boost::function<void(std::vector<std::string>&)> callback)
  {
    while(!m_smt.checkSVField(field_name)) //wait for the field to be advertised
    {
      //TODO: add a warning?
      usleep(10000);
    }

    while(!m_shutdown)
    {
      m_smt.awaitNewData(field_name);

      std::vector<std::string> data;
      m_smt.getSVField(field_name, data);
      callback(data);
      m_smt.signalProcessed(field_name);
    }
  }
}
