#include "shared_memory_interface/shared_memory_interface.hpp"

namespace shared_memory_interface
{
  SharedMemoryInterface::SharedMemoryInterface(std::string interface_name) :
      m_smt(interface_name)
  {
    m_shutdown = false;
    m_interface_name = interface_name;
    m_callback_thread = new boost::thread(boost::bind(&SharedMemoryInterface::callbackThreadFunction, this));
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

  //callback
  void SharedMemoryInterface::callbackThreadFunction()
  {
    //TODO: use conditional variables to improve efficiency
    //TODO: add ability to get current value even if not new
    //TODO: attempt to merge all the subscription maps
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
          m_smt.signalProcessed(iter->first);
        }
      }

      typedef std::map<std::string, boost::function<void(std::vector<std::string>&)> >::iterator StringMapIter;
      for(StringMapIter iter = m_string_subscriptions.begin(); iter != m_string_subscriptions.end(); iter++)
      {
        if(m_smt.hasNew(iter->first))
        {
          std::vector<std::string> data;
          m_smt.getSVField(iter->first, data);
          iter->second(data);
          m_smt.signalProcessed(iter->first);
        }
      }

#if(USE_ROS)
      typedef std::map<std::string, SubscriptionInfo>::iterator SerializedMapIter;
      for(SerializedMapIter iter = m_serialized_subscriptions.begin(); iter != m_serialized_subscriptions.end(); iter++)
      {
        if(m_smt.hasNew(iter->first))
        {
          std::cerr << iter->first << " has new data!\n";
          std::string serialized_data;
          m_smt.getSerializedField(iter->first, serialized_data, iter->second.md5sum, iter->second.datatype);

          boost::shared_array<unsigned char> buf((unsigned char*) (const_cast<char *>(serialized_data.c_str())));
          ros::SerializedMessage serialized_msg(buf, serialized_data.length());
          ros::MessageDeserializerPtr deserializer(new ros::MessageDeserializer(iter->second.helper, serialized_msg, boost::shared_ptr<ros::M_string>()));

          ros::VoidConstPtr msg = deserializer->deserialize();
          if(msg)
          {
            ros::SubscriptionCallbackHelperCallParams params;
            params.event = ros::MessageEvent<void const>(msg, deserializer->getConnectionHeader(), ros::Time(0), true, ros::MessageEvent<void const>::CreateFunction());
            iter->second.helper->call(params);
            m_smt.signalProcessed(iter->first);
          }
        }
      }
#endif

    }
  }
}
