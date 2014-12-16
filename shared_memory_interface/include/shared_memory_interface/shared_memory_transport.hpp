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

#ifndef SHARED_MEMORY_TRANSPORT_HPP
#define SHARED_MEMORY_TRANSPORT_HPP

#include "shared_memory_utils.hpp"

namespace shared_memory_interface
{
  template<typename T> //T must be the type of a ros message
  class SharedMemoryTransport
  {
  public:
    SharedMemoryTransport(unsigned long reservation_size = (2*1048576));//2 Mebibytes
    ~SharedMemoryTransport();

    static bool createMemory(std::string interface_name, unsigned int size);
    static void destroyMemory(std::string interface_name);

    bool initialized();
    bool connected();
    void configure(std::string interface_name, std::string field_name, bool create_field = false);
    bool connect(double timeout = 0.0);
    bool createField();
    bool getData(T& data);
    bool setData(T& data);

    std::string getFieldName();

    bool hasData(); //returns true if the field has already been configured
    bool awaitNewDataPolled(T& data, double timeout = -1);
    bool awaitNewData(T& data, double timeout = -1);

  private:
    boost::interprocess::managed_shared_memory* segment;
    boost::thread* m_watchdog_thread;
    void watchdogFunction();

    unsigned long m_reservation_size;

    bool m_initialized;
    bool m_connected;
    std::string m_interface_name;
    std::string m_field_name;
    std::string m_even_buffer_name;
    std::string m_even_length_name;
    std::string m_odd_buffer_name;
    std::string m_odd_length_name;
    std::string m_buffer_sequence_id_name;
    std::string m_condition_name;
    std::string m_condition_mutex_name;
    std::string m_invalid_flag_name;
    std::string m_exists_flag_name;

    SMCharAllocator* m_string_allocator;

    uint32_t* m_buffer_sequence_id_ptr;
    bool* m_invalid_ptr;
    SMString* m_even_string_ptr;
    unsigned char* m_even_data_ptr;
    uint32_t* m_even_length_ptr;
    SMString* m_odd_string_ptr;
    unsigned char* m_odd_data_ptr;
    uint32_t* m_odd_length_ptr;
    boost::interprocess::interprocess_condition* m_condition_ptr;
    boost::interprocess::interprocess_mutex* m_condition_mutex_ptr;

    //remembered flags
    bool m_already_read_valid;
    bool m_already_set_valid;

    uint32_t m_last_read_buffer_sequence_id;
  };

}
#endif //SHARED_MEMORY_TRANSPORT_HPP
