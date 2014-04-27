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

#ifndef SHARED_MEMORY_INTERFACE_ROS_HPP
#define SHARED_MEMORY_INTERFACE_ROS_HPP

#include "shared_memory_interface/shared_memory_interface.hpp"

#include <ros/serialization.h>
#include <ros/parameter_adapter.h>
#include <ros/subscription_callback_helper.h>
#include <ros/message_deserializer.h>

namespace shared_memory_interface
{
  class SharedMemoryInterfaceROS: public SharedMemoryInterface
  {
  public:
    SharedMemoryInterfaceROS(std::string interface_name) :
        SharedMemoryInterface(interface_name)
    {
    }

    ~SharedMemoryInterfaceROS()
    {

    }

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

    template<typename T> //T must be the type of a ros message
    bool subscribeSerializedROS(std::string field_name, boost::function<void(T&)> callback)
    {
      m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterfaceROS::callbackThreadFunctionSerialized<T>, this, field_name, callback)));

      return true;
    }

  protected:
    template<typename T> //T must be the type of a ros message
    void callbackThreadFunctionSerialized(std::string field_name, boost::function<void(T&)> callback)
    {
      while(!m_smt.checkSerializedField(field_name) && ros::ok()) //wait for the field to be advertised
      {
        //TODO: add a warning?
        usleep(10000);
      }

      typedef typename ros::ParameterAdapter<T>::Message MessageType;
      while(!m_shutdown && ros::ok())
      {
        m_smt.awaitNewData(field_name);

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
  };

}
#endif //SHARED_MEMORY_INTERFACE_ROS_HPP
