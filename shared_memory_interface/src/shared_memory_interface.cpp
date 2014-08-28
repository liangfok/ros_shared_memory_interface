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
    return m_smt.addStringVectorField(field_name, length);
  }

  bool SharedMemoryInterface::advertiseFloatingPointVector(std::string field_name, unsigned long length)
  {
    return m_smt.addFloatingPointMatrixField(field_name, 1, length);
  }

  bool SharedMemoryInterface::advertiseFloatingPointMatrix(std::string field_name, unsigned long rows, unsigned long cols)
  {
    return m_smt.addFloatingPointMatrixField(field_name, rows, cols);
  }

  bool SharedMemoryInterface::publishStringVector(std::string field_name, std::vector<std::string>& data)
  {
    if(m_smt.setStringVectorField(field_name, data))
    {
      if(m_smt.signalAvailable(field_name))
      {
        return true;
      }
    }
    return false;
  }

  bool SharedMemoryInterface::publishFloatingPointVector(std::string field_name, std::vector<double>& data)
  {
    if(m_smt.setFloatingPointField(field_name, data))
    {
      if(m_smt.signalAvailable(field_name))
      {
        return true;
      }
    }
    return false;
  }

  bool SharedMemoryInterface::publishFloatingPointMatrix(std::string field_name, std::vector<double>& data)
  {
    if(m_smt.setFloatingPointField(field_name, data))
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
    m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterface::callbackThreadFunctionStringVector, this, field_name, callback)));
    return true;
  }

  bool SharedMemoryInterface::subscribeFloatingPointVector(std::string field_name, boost::function<void(std::vector<double>&)> callback)
  {
    m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterface::callbackThreadFunctionFloatingPoint, this, field_name, callback)));
    return true;
  }

  bool SharedMemoryInterface::subscribeFloatingPointMatrix(std::string field_name, boost::function<void(std::vector<double>&)> callback)
  {
    m_callback_threads.push_back(new boost::thread(boost::bind(&SharedMemoryInterface::callbackThreadFunctionFloatingPoint, this, field_name, callback)));
    return true;
  }

  bool SharedMemoryInterface::waitForStringVector(std::string field_name, std::vector<std::string>& data, double timeout)
  {
    if(!m_smt.awaitNewData(field_name, timeout))
    {
      return false;
    }
    m_smt.getStringVectorField(field_name, data);
    m_smt.signalProcessed(field_name);
    return true;
  }

  bool SharedMemoryInterface::waitForFloatingPointVector(std::string field_name, std::vector<double>& data, double timeout)
  {
    if(!m_smt.awaitNewData(field_name, timeout))
    {
      return false;
    }
    m_smt.getFloatingPointField(field_name, data);
    m_smt.signalProcessed(field_name);
    return true;
  }

  bool SharedMemoryInterface::waitForFloatingPointMatrix(std::string field_name, std::vector<double>& data, double timeout)
  {
    if(!m_smt.awaitNewData(field_name, timeout))
    {
      return false;
    }
    m_smt.getFloatingPointField(field_name, data);
    m_smt.signalProcessed(field_name);
    return true;
  }

  bool SharedMemoryInterface::getCurrentStringVector(std::string field_name, std::vector<std::string>& data, double timeout)
  {
    if(!m_smt.checkStringVectorField(field_name)) //if the field doesn't exist yet, we can't retrieve the data
    {
      if(!m_smt.awaitNewData(field_name, timeout)) //wait for the field to be advertised
      {
        return false;
      }
    }
    m_smt.getStringVectorField(field_name, data);
    return true;
  }

  bool SharedMemoryInterface::getCurrentFloatingPointVector(std::string field_name, std::vector<double>& data, double timeout)
  {
    if(!m_smt.checkFloatingPointField(field_name)) //if the field doesn't exist yet, we can't retrieve the data
    {
      if(!m_smt.awaitNewData(field_name, timeout)) //wait for the field to be advertised
      {
        return false;
      }
    }
    m_smt.getFloatingPointField(field_name, data);
    return true;
  }

  bool SharedMemoryInterface::getCurrentFloatingPointMatrix(std::string field_name, std::vector<double>& data, double timeout)
  {
    if(!m_smt.checkFloatingPointField(field_name)) //if the field doesn't exist yet, we can't retrieve the data
    {
      if(!m_smt.awaitNewData(field_name, timeout)) //wait for the field to be advertised
      {
        return false;
      }
    }
    m_smt.getFloatingPointField(field_name, data);
    return true;
  }

  //private callback threads
  void SharedMemoryInterface::callbackThreadFunctionFloatingPoint(std::string field_name, boost::function<void(std::vector<double>&)> callback)
  {
    while(!m_smt.checkFloatingPointField(field_name)) //wait for the field to be advertised
    {
      std::cerr << "SharedMemoryInterface::callbackThreadFunctionFloatingPoint: Waiting for " << field_name << " to be advertised" << std::endl;
      usleep(1000000);
    }

    while(!m_shutdown)
    {
      m_smt.awaitNewData(field_name);

      std::vector<double> data;
      m_smt.getFloatingPointField(field_name, data);
      callback(data);
      m_smt.signalProcessed(field_name);
    }
  }

  void SharedMemoryInterface::callbackThreadFunctionStringVector(std::string field_name, boost::function<void(std::vector<std::string>&)> callback)
  {
    while(!m_smt.checkStringVectorField(field_name)) //wait for the field to be advertised
    {
      std::cerr << "SharedMemoryInterface::callbackThreadFunctionStringVector: Waiting for " << field_name << " to be advertised" << std::endl;
      usleep(1000000);
    }

    while(!m_shutdown)
    {
      m_smt.awaitNewData(field_name);

      std::vector<std::string> data;
      m_smt.getStringVectorField(field_name, data);
      callback(data);
      m_smt.signalProcessed(field_name);
    }
  }
}
