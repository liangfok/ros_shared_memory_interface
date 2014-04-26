#ifndef SHARED_MEMORY_INTERFACE_HPP
#define SHARED_MEMORY_INTERFACE_HPP

#include "shared_memory_interface/shared_memory_transport.hpp"

#include <vector>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/type_traits.hpp>

#define USE_ROS 1

#if(USE_ROS)
#include <ros/serialization.h>
#include <ros/parameter_adapter.h>
#include <ros/subscription_callback_helper.h>
#include <ros/message_deserializer.h>
#endif

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

    bool subscribeStringVector(std::string field, boost::function<void(std::vector<std::string>&)> callback);
    bool subscribeFPVector(std::string field, boost::function<void(std::vector<double>&)> callback);
    bool subscribeFPMatrix(std::string field, boost::function<void(std::vector<double>&)> callback);

#if(USE_ROS) //generic implementations, works with any ROS message type but may be slower
    struct SubscriptionInfo
    {
      std::string md5sum;
      std::string datatype;
      ros::SubscriptionCallbackHelperPtr helper; //contains information needed to deserialize
    };

    template<typename T> //T must be the type of a ros message
    bool advertiseSerializedROS(std::string field_name)
    {
      typedef typename ros::ParameterAdapter<T>::Message MessageType;
      std::string md5sum = ros::message_traits::md5sum<MessageType>();
      std::string datatype = ros::message_traits::datatype<MessageType>();

      return m_smt.addSerializedField(field_name, md5sum, datatype);
    }

    template<typename T> //T must be the type of a ros message
    bool publishSerializedROS(std::string field_name, T& data)
    {
      typedef typename ros::ParameterAdapter<T>::Message MessageType;
      std::string md5sum = ros::message_traits::md5sum<MessageType>();
      std::string datatype = ros::message_traits::datatype<MessageType>();
      std::string serialized;

      unsigned long oserial_size = ros::serialization::serializationLength(data);
      serialized.resize(oserial_size);

      ros::serialization::OStream ostream((unsigned char*) &serialized[0], oserial_size);
      ros::serialization::serialize(ostream, data);

      if(m_smt.setSerializedField(field_name, serialized, md5sum, datatype))
      {
        if(m_smt.signalAvailable(field_name))
        {
          return true;
        }
      }
      return false;
    }

    //TODO: move to private
    template<typename T> //T must be the type of a ros message
    void serializedCallbackThreadFunction(std::string field_name, boost::function<void(T&)> callback)
    {
      typedef typename ros::ParameterAdapter<T>::Message MessageType;
      while(true)
      {
        if(m_smt.hasNew(field_name))
        {
          std::string serialized_data, md5sum, datatype;
          m_smt.getSerializedField(field_name, serialized_data, md5sum, datatype);
          m_smt.signalProcessed(field_name);

          if(serialized_data == "")
          {
            std::cerr << "Serial data empty even though flag was set!";
            continue;
          }

          T msg;
          ros::serialization::IStream istream((uint8_t*) &serialized_data[0], serialized_data.size());
          ros::serialization::deserialize(istream, msg);
          callback(msg);
        }
      }
    }

    template<typename T> //T must be the type of a ros message
    bool subscribeSerializedROS(std::string field_name, boost::function<void(T&)> callback)
    {
      m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterface::serializedCallbackThreadFunction<T>, this, field_name, callback)));

//      typedef typename ros::ParameterAdapter<T>::Message MessageType;
//      SubscriptionInfo sub_info;
//      sub_info.md5sum = ros::message_traits::md5sum<MessageType>();
//      sub_info.datatype = ros::message_traits::datatype<MessageType>();
//      sub_info.helper = ros::SubscriptionCallbackHelperPtr(new ros::SubscriptionCallbackHelperT<const boost::shared_ptr<MessageType const>&>(callback, ros::DefaultMessageCreator<T>()));
//
//      m_serialized_subscriptions[field_name] = sub_info;

      return true;
    }
#endif

  private:
    SharedMemoryTransport m_smt;
    std::string m_interface_name;
    std::map<std::string, boost::function<void(std::vector<double>&)> > m_FP_subscriptions;
    std::map<std::string, boost::function<void(std::vector<std::string>&)> > m_string_subscriptions;

#if(USE_ROS)
    std::map<std::string, SubscriptionInfo> m_serialized_subscriptions;
#endif

    boost::thread* m_callback_thread;
    std::vector<boost::thread*> m_callback_threads;
    bool m_shutdown;

    void callbackThreadFunction();
  };

}
#endif //SHARED_MEMORY_INTERFACE_HPP
