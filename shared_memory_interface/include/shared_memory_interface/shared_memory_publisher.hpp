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

#ifndef SHARED_MEMORY_PUBLISHER_HPP
#define SHARED_MEMORY_PUBLISHER_HPP

#include "shared_memory_transport.hpp"

namespace shared_memory_interface
{
  template<typename T> //T must be the type of a ros message
  class Publisher
  {
  public:
    Publisher(bool write_to_rostopic = true)
    {
      m_write_to_rostopic = write_to_rostopic;
    }

    ~Publisher()
    {
    }

    void advertise(std::string topic_name, std::string shared_memory_interface_name = "smi")
    {
      m_interface_name = shared_memory_interface_name;
      configureTopicPaths(m_interface_name, topic_name, m_full_ros_topic_path, m_full_topic_path);
      m_smt.configure(m_interface_name, m_full_topic_path);
      m_smt.createField();

      if(m_write_to_rostopic)
      {
        ros::NodeHandle nh("~");
        m_ros_publisher = nh.advertise<T>(m_full_ros_topic_path, 1, true);
      }
    }

    bool publish(T& data)
    {
      std::string serialized;

      unsigned long oserial_size = ros::serialization::serializationLength(data);
      serialized.resize(oserial_size);

      ros::serialization::OStream ostream((unsigned char*) &serialized[0], oserial_size);
      ros::serialization::serialize(ostream, data);

      if(m_smt.setData(serialized))
      {
        if(m_write_to_rostopic) // && m_ros_publisher.getNumSubscribers() > 0)
        {
          m_ros_publisher.publish(data);
        }
        return true;
      }
      else
      {
        ROS_ERROR("Failed to write to topic %s!", m_full_topic_path.c_str());
        return false;
      }
    }

  protected:
    SharedMemoryTransport m_smt;

    std::string m_interface_name;
    std::string m_full_topic_path;
    std::string m_full_ros_topic_path;

    bool m_write_to_rostopic;
    ros::Publisher m_ros_publisher;
  };

}
#endif //SHARED_MEMORY_PUBLISHER_HPP
