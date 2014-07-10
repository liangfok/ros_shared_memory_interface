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

#include <algorithm>

namespace shared_memory_interface
{
  class SharedMemoryInterfaceROS: public SharedMemoryInterface
  {
  public:
    SharedMemoryInterfaceROS(std::string interface_name, bool do_pub = true) :
        SharedMemoryInterface(interface_name), m_nh("~")
    {
      m_do_pub = do_pub;
    }

    ~SharedMemoryInterfaceROS()
    {
    }

    template<typename T> //T must be the type of a ros message
    bool advertiseSerializedROS(std::string field_name)
    {
      std::string ros_field_name, sm_field_name;
      getFullNames(field_name, ros_field_name, sm_field_name);

      typedef typename ros::ParameterAdapter<T>::Message MessageType;
      std::string md5sum = ros::message_traits::md5sum<MessageType>();
      std::string datatype = ros::message_traits::datatype<MessageType>();

      m_pub_map[ros_field_name] = m_nh.advertise<T>(ros_field_name, 1, true);
      return m_smt.addSerializedField(sm_field_name, md5sum, datatype);
    }

    template<typename T> //T must be the type of a ros message
    bool publishSerializedROS(std::string field_name, T& data)
    {
      std::string ros_field_name, sm_field_name;
      getFullNames(field_name, ros_field_name, sm_field_name);

      typedef typename ros::ParameterAdapter<T>::Message MessageType;
      std::string md5sum = ros::message_traits::md5sum<MessageType>();
      std::string datatype = ros::message_traits::datatype<MessageType>();
      std::string serialized;

      unsigned long oserial_size = ros::serialization::serializationLength(data);
      serialized.resize(oserial_size);

      ros::serialization::OStream ostream((unsigned char*) &serialized[0], oserial_size);
      ros::serialization::serialize(ostream, data);

      if(m_smt.setSerializedField(sm_field_name, serialized, md5sum, datatype))
      {
        if(m_smt.signalAvailable(sm_field_name))
        {
          if(m_do_pub) // && m_pub_map[field_name].getNumSubscribers() > 0)
          {
            m_pub_map[ros_field_name].publish(data);
          }
          return true;
        }
        else
        {
          std::cerr << "Failed to signal available on field " << sm_field_name << std::endl;
          return false;
        }
      }
      else
      {
        std::cerr << "Failed to write to field " << sm_field_name << std::endl;
        return false;
      }
    }

    template<typename T> //T must be the type of a ros message
    bool subscribeSerializedROS(std::string field_name, boost::function<void(T&)> callback)
    {
      std::string ros_field_name, sm_field_name;
      getFullNames(field_name, ros_field_name, sm_field_name);

      m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterfaceROS::callbackThreadFunctionSerialized<T>, this, sm_field_name, callback)));
      m_sub_map[ros_field_name] = m_nh.subscribe<T>(ros_field_name, 1, boost::bind(&SharedMemoryInterfaceROS::blankCallback<T>, this, _1));
      return true;
    }

    template<typename T> //T must be the type of a ros message
    bool waitForSerializedROS(std::string field_name, T& msg, double timeout = -1)
    {
      std::string ros_field_name, sm_field_name;
      getFullNames(field_name, ros_field_name, sm_field_name);

      if(!m_smt.awaitNewData(sm_field_name, timeout))
      {
        return false;
      }
      std::string serialized_data, md5sum, datatype;
      m_smt.getSerializedField(sm_field_name, serialized_data, md5sum, datatype);
      m_smt.signalProcessed(sm_field_name);

      ros::serialization::IStream istream((uint8_t*) &serialized_data[0], serialized_data.size());
      ros::serialization::deserialize(istream, msg);
      return true;
    }

    template<typename T> //T must be the type of a ros message
    bool getCurrentSerializedROS(std::string field_name, T& msg, double timeout = -1)
    {
      std::string ros_field_name, sm_field_name;
      getFullNames(field_name, ros_field_name, sm_field_name);

      if(timeout == 0 && !m_smt.checkSerializedField(sm_field_name))
      {
        return false;
      }
      else if(timeout < 0)
      {
        //WARNING: the await functions appear to not work on some machines.
        m_smt.awaitNewData(sm_field_name);
      }
      else
      {
        boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(1000.0 * timeout);
        while(!m_smt.checkSerializedField(sm_field_name)) //if the field doesn't exist yet, we can't retrieve the data
        {
          if(boost::get_system_time() > timeout_time) //wait for the field to be advertised
          {
            return false;
          }
        }
      }

      if(m_sub_map.find(ros_field_name) == m_sub_map.end())
      {
        m_sub_map[ros_field_name] = m_nh.subscribe<T>(ros_field_name, 1, boost::bind(&SharedMemoryInterfaceROS::blankCallback<T>, this, _1));
      }

      std::string serialized_data, md5sum, datatype;
      m_smt.getSerializedField(sm_field_name, serialized_data, md5sum, datatype);
      m_smt.signalProcessed(sm_field_name);

      if(serialized_data.length() == 0)
      {
        ROS_WARN_THROTTLE(1.0, "SM field %s exists but has never been written to!", sm_field_name.c_str());
        return false;
      }

      ros::serialization::IStream istream((uint8_t*) &serialized_data[0], serialized_data.size());
      ros::serialization::deserialize(istream, msg);
      return true;
    }

  protected:
    template<typename T> //T must be the type of a ros message
    void callbackThreadFunctionSerialized(std::string field_name, boost::function<void(T&)> callback)
    {
      while(!m_smt.checkSerializedField(field_name) && ros::ok()) //wait for the field to be advertised
      {
        std::cerr << "Waiting for " << field_name << " to be advertised." << std::endl;
        usleep(1000000);
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

    template<typename T> //T must be the type of a ros message
    void blankCallback(const typename T::ConstPtr& msg)
    {
    }

    void getFullNames(std::string field_name, std::string& ros_field_name, std::string& sm_field_name)
    {
      sm_field_name = m_nh.resolveName(field_name).substr(1);
      ros_field_name = "/" + m_interface_name + "/" + sm_field_name;
      std::replace(sm_field_name.begin(), sm_field_name.end(), '/', '-'); //convert slashes for boost compatibility
    }

    ros::NodeHandle m_nh;
    std::map<std::string, ros::Subscriber> m_sub_map;
    std::map<std::string, ros::Publisher> m_pub_map;
    bool m_do_pub;
  };

}
#endif //SHARED_MEMORY_INTERFACE_ROS_HPP
