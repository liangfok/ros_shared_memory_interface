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

#include "shared_memory_transport_impl.hpp"

namespace shared_memory_interface
{
  template<typename T> //T must be the type of a ros message
  class Publisher
  {
  public:
    Publisher(bool write_to_rostopic = true)
    {
      m_nh = NULL;
      m_write_to_rostopic = write_to_rostopic;
      advertised = false;
    }

    ~Publisher()
    {
    }

    void advertise(std::string topic_name, std::string shared_memory_interface_name = "smi")
    {
      m_interface_name = shared_memory_interface_name;
      configureTopicPaths(m_interface_name, topic_name, m_full_ros_topic_path, m_full_topic_path);
      m_smt.configure(m_interface_name, m_full_topic_path, true);
      assert(m_smt.connect()); //connection CANNOT fail, since we literally just created the field

      if(m_write_to_rostopic)
      {
        ros::NodeHandle nh("~");
        m_ros_publisher = nh.advertise<T>(m_full_ros_topic_path, 1, true);
      }
      advertised = true;
    }

    bool publish(T& data)
    {
      if(!m_nh)
      {
        m_nh = new ros::NodeHandle("~");
      }
      if(!m_smt.connected())
      {
        if(!m_smt.connect())
        {
          ROS_WARN_THROTTLE(1.0, "%s: Tried to publish on an unconfigured shared memory publisher: %s!", m_nh->getNamespace().c_str(), m_full_topic_path.c_str());
          assert(advertised);
          return false;
        }
      }

      if(m_smt.setData(data))
      {
        if(m_write_to_rostopic) // && m_ros_publisher.getNumSubscribers() > 0)
        {
          m_ros_publisher.publish(data);
        }
        return true;
      }
      else
      {
        ROS_ERROR("%s: Failed to write to topic %s!", m_nh->getNamespace().c_str(), m_full_topic_path.c_str());
        return false;
      }
    }

  protected:
    ros::NodeHandle* m_nh;
    SharedMemoryTransport<T> m_smt;
    bool advertised;

    std::string m_interface_name;
    std::string m_full_topic_path;
    std::string m_full_ros_topic_path;

    bool m_write_to_rostopic;
    ros::Publisher m_ros_publisher;
  };

}
#endif //SHARED_MEMORY_PUBLISHER_HPP
