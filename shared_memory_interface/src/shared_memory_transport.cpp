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

#include "shared_memory_interface/shared_memory_transport.hpp"

namespace shared_memory_interface
{
#define TRACE 0
#define PRINT_TRACE_ENTER if(TRACE)std::cerr<<__func__<<std::endl;
#define PRINT_TRACE_EXIT if(TRACE)std::cerr<<"/"<<__func__<<std::endl;

  typedef boost::interprocess::allocator<double, boost::interprocess::managed_shared_memory::segment_manager> SMDoubleAllocator;
  typedef boost::interprocess::vector<double, SMDoubleAllocator> SMDoubleVector;

  typedef boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> SMCharAllocator;
  typedef boost::interprocess::basic_string<char, std::char_traits<char>, SMCharAllocator> SMString;
  typedef boost::interprocess::allocator<SMString, boost::interprocess::managed_shared_memory::segment_manager> SMStringAllocator;
  typedef boost::interprocess::vector<SMString, SMStringAllocator> SMStringVector;

  boost::interprocess::permissions unrestricted()
  {
    boost::interprocess::permissions perm;
    perm.set_unrestricted();
    return perm;
  }

  class SMScopedLock
  {
  public:
    SMScopedLock(std::string interface_name, std::string field_name)
    {
      assert(interface_name.length() != 0);
      m_interface_name = interface_name;
      m_field_name = field_name;
      m_full_name = getFullName(m_interface_name, m_field_name);
      try
      {
        //std::cerr << "Attached to mutex " << m_full_name << "!" << std::endl;
        m_mutex = new boost::interprocess::named_upgradable_mutex(boost::interprocess::open_only, m_full_name.c_str());
        
      }
      catch(...)
      {
        std::cerr << "Mutex " << m_full_name << " not found! Creating it!";
        m_mutex = new boost::interprocess::named_upgradable_mutex(boost::interprocess::open_or_create, m_full_name.c_str(), unrestricted());
        //m_mutex = new boost::interprocess::named_upgradable_mutex(boost::interprocess::open_or_create, m_full_name.c_str()); //TODO: figure out why this crashes
      }
    }

    ~SMScopedLock()
    {
      delete m_mutex;
    }

    boost::interprocess::named_upgradable_mutex* get()
    {
      return m_mutex;
    }

    static void destroy(std::string interface_name, std::string field_name)
    {
      assert(interface_name.length() != 0);
      boost::interprocess::named_upgradable_mutex::remove(getFullName(interface_name, field_name).c_str());
    }

    static void create(std::string interface_name, std::string field_name)
    {
      assert(interface_name.length() != 0);
      boost::interprocess::named_upgradable_mutex(boost::interprocess::open_or_create, getFullName(interface_name, field_name).c_str(), unrestricted());
    }

    static std::string getFullName(std::string& interface_name, std::string field_name)
    {
      assert(interface_name.length() != 0);
      //std::cerr << interface_name << "+" << field_name << "+mutex=" << (interface_name + field_name + "mutex") << std::endl;
      return (interface_name + "__" + field_name + "__mutex");
    }

  protected:
    void repairBrokenLock()
    {
      std::cerr << "SharedMemoryTransport: Found broken lock " << m_full_name << "! Repairing!" << std::endl;
      assert(m_interface_name.length() != 0);
      destroy(m_interface_name, m_field_name);
      create(m_interface_name, m_field_name);
      delete m_mutex;
      m_mutex = new boost::interprocess::named_upgradable_mutex(boost::interprocess::open_only, m_full_name.c_str());
    }

    boost::interprocess::named_upgradable_mutex* m_mutex;
    std::string m_interface_name;
    std::string m_field_name;
    std::string m_full_name;
  };

  class SMScopedReaderLock: public SMScopedLock
  {
  public:
    SMScopedReaderLock(std::string interface_name, std::string field_name, double timeout_duration = 1.0) :
        SMScopedLock(interface_name, field_name)
    {
      boost::posix_time::ptime timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout_duration * 1000);
      if(!m_mutex->timed_lock_sharable(timeout))
      {
        repairBrokenLock();
      }
    }

    ~SMScopedReaderLock()
    {
      m_mutex->unlock_sharable();
    }
  };

  class SMScopedWriterLock: public SMScopedLock
  {
  public:
    SMScopedWriterLock(std::string interface_name, std::string field_name, double timeout_duration = 1.0) :
        SMScopedLock(interface_name, field_name)
    {
      boost::posix_time::ptime timeout = boost::get_system_time() + boost::posix_time::milliseconds(timeout_duration * 1000);
      if(!m_mutex->timed_lock(timeout))
      {
        repairBrokenLock();
      }
    }

    ~SMScopedWriterLock()
    {
      m_mutex->unlock();
    }
  };

  SharedMemoryTransport::SharedMemoryTransport(std::string interface_name)
  {
    PRINT_TRACE_ENTER
    std::cerr << "Initializing SharedMemoryTransport" << std::endl;

    char *name;
    struct passwd *pass;
    pass = getpwuid(getuid());
    name = pass->pw_name;
    std::string username = std::string(name);

//    char buf[100];
//    getlogin_r(buf, 100);
//    std::string username = std::string(buf);
    m_interface_name = username + "__" + interface_name;
    m_data_name = m_interface_name + "data";

    SMScopedWriterLock memory_lock(m_interface_name, "");

    //try to open existing shared memory
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      unsigned long* connection_tokens = segment.find<unsigned long>("connection_tokens").first;
      *connection_tokens = *connection_tokens + 1;
      std::cerr << "Connected to " << m_interface_name << " space." << std::endl;
    }
    catch(boost::interprocess::interprocess_exception &ex) //shared memory hasn't been created yet, so we'll make it
    {
      std::cerr << "SM space " << m_interface_name << " not found. Creating it." << std::endl;
      unsigned long base_size = 8192; //make sure we have enough room to create the default objects
      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, m_data_name.c_str(), base_size, NULL, unrestricted());

        unsigned long* connection_tokens = segment.construct<unsigned long>("connection_tokens")(0);
        *connection_tokens = *connection_tokens + 1;
      }
      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str());
      std::cerr << "Created " << m_interface_name << " space." << std::endl;
    }
    PRINT_TRACE_EXIT
  }

  SharedMemoryTransport::~SharedMemoryTransport()
  {
    //TODO: find a way to ensure that the destructor is called
    {
      SMScopedWriterLock memory_lock(m_interface_name, "");
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      unsigned long* connection_tokens = segment.find<unsigned long>("connection_tokens").first;
      if((*connection_tokens) > 1) //we aren't the last one here
      {
        std::cerr << "Disconnecting from shared memory.\n";
        *connection_tokens = *connection_tokens - 1;
        return;
      }
    }
    //if we have reached this point, we are the last one attached to the memory, so delete everything
    std::cerr << "Closing down shared memory cleanly.\n";
    boost::interprocess::shared_memory_object::remove(m_data_name.c_str());
    SMScopedLock::destroy(m_interface_name, "");
  }

  void SharedMemoryTransport::destroyMemory(std::string interface_name)
  {
    char *name;
    struct passwd *pass;
    pass = getpwuid(getuid());
    name = pass->pw_name;
    std::string username = std::string(name);
    std::string full_interface_name = username + "__" + interface_name;

    std::cerr << "Destroying shared memory space " << full_interface_name << "." << std::endl;
    std::string data_name = full_interface_name + "data";
    boost::interprocess::shared_memory_object::remove(data_name.c_str());
    SMScopedLock::destroy(full_interface_name, "");
  }

  bool SharedMemoryTransport::addFloatingPointMatrixField(std::string field_name, unsigned long rows, unsigned long cols)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock memory_lock(m_interface_name, "");

    //check to see if someone else created the field_name
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      SMDoubleVector* vector = segment.find<SMDoubleVector>(field_name.c_str()).first;
      if(vector) //field_name already created
      {
        if(vector->size() == (rows * cols))
        {
          std::cerr << "Using existing SM field for " << field_name << std::endl;
          PRINT_TRACE_EXIT
          return true; //field_name already exists, so we're done
        }

        //field_name isn't the right size, need to recreate everything
        std::cerr << "WARNING: replacing existing field_name!";
        segment.destroy<SMDoubleVector>(field_name.c_str());
        segment.destroy<bool>((field_name + "_new_data_flag").c_str());
        segment.destroy<bool>(std::string(field_name + "_invalid").c_str());
        segment.destroy<unsigned long>((field_name + "_row_stride").c_str());
        boost::interprocess::named_upgradable_mutex::remove((m_interface_name + field_name + "mutex").c_str());
        boost::interprocess::named_upgradable_mutex(boost::interprocess::open_or_create, (m_interface_name + field_name + "mutex").c_str(), unrestricted());
      }
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while testing existance of field_name \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    //no one did, so we'll make it ourselves
    try
    {
      unsigned long size = rows * cols * sizeof(double) + 4096;
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), size); //add space for the new field_name's data + overhead
      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
        const SMDoubleAllocator alloc_double_inst(segment.get_segment_manager());
        SMDoubleVector* vector = segment.construct<SMDoubleVector>(field_name.c_str())(alloc_double_inst);
        vector->resize(rows * cols, 0.0);
        segment.construct<bool>(std::string(field_name + "_new_data_flag").c_str())(false);
        segment.construct<bool>(std::string(field_name + "_invalid").c_str())(true); //field is invalid until someone writes actual data to it
        segment.construct<unsigned long>((field_name + "_row_stride").c_str())(cols); //row_stride = number of cols in each row
        SMScopedLock::destroy(m_interface_name, field_name); //this shouldn't be necessary, but sometimes helps
        SMScopedLock::create(m_interface_name, field_name);
      }
      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while creating new field_name \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::addStringVectorField(std::string field_name, unsigned long length)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock memory_lock(m_interface_name, "");

    //check to see if someone else created the field_name
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      SMStringVector* vector = segment.find<SMStringVector>(field_name.c_str()).first;
      if(vector) //field_name already created
      {
        if(vector->size() == length)
        {
          std::cerr << "Using existing SM field for " << field_name << std::endl;
          PRINT_TRACE_EXIT
          return true; //field_name already exists, so we're done
        }

        //field_name isn't the right size, need to recreate everything
        std::cerr << "WARNING: replacing existing field_name!";
        segment.destroy<SMStringVector>(field_name.c_str());
        segment.destroy<bool>((field_name + "_new_data_flag").c_str());
        segment.destroy<bool>(std::string(field_name + "_invalid").c_str());
      }
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while testing existance of field_name \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    //no one did, so we'll make it ourselves
    try
    {
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), length * sizeof(char) + 4096); //add space for the new field_name's data + overhead

      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());

        const SMCharAllocator alloc_char_inst(segment.get_segment_manager());
        SMStringVector* strings_sm = segment.construct<SMStringVector>(field_name.c_str())(segment.get_segment_manager());
        for(unsigned long i = 0; i < length; i++)
        {
          SMString string(alloc_char_inst);
          string = "";
          strings_sm->push_back(string);
        }
        segment.construct<bool>(std::string(field_name + "_new_data_flag").c_str())(false);
        segment.construct<bool>(std::string(field_name + "_invalid").c_str())(true); //field is invalid until someone writes actual data to it
        SMScopedLock::destroy(m_interface_name, field_name); //this shouldn't be necessary, but sometimes helps
        SMScopedLock::create(m_interface_name, field_name);
      }

      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while creating new field_name \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::addSerializedField(std::string field_name, std::string md5sum, std::string datatype)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock memory_lock(m_interface_name, "");

    //check to see if someone else created the field_name
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      SMString* string = segment.find<SMString>(field_name.c_str()).first;
      if(string) //field_name already created
      {
        std::cerr << "Using existing SM field for " << field_name << std::endl;
        PRINT_TRACE_EXIT
        return true; //field_name already exists, so we're done
      }
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while testing existance of field_name \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    //no one did, so we'll make it ourselves
    try
    {
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), 4096); //add space for the new field_name's data + overhead

      {
        std::cerr << "Creating new SM field for " << field_name << std::endl;
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());

        const SMCharAllocator alloc_char_inst(segment.get_segment_manager());
        SMString* string_sm = segment.construct<SMString>(field_name.c_str())(alloc_char_inst);
        *string_sm = "";

        SMString* md5sum_sm = segment.construct<SMString>((field_name + "_md5sum").c_str())(alloc_char_inst);
        SMString* datatype_sm = segment.construct<SMString>((field_name + "_datatype").c_str())(alloc_char_inst);
        *md5sum_sm = md5sum.c_str();
        *datatype_sm = datatype.c_str();

        segment.construct<bool>(std::string(field_name + "_new_data_flag").c_str())(false);
        segment.construct<bool>(std::string(field_name + "_invalid").c_str())(true); //field is invalid until someone writes actual data to it
        SMScopedLock::destroy(m_interface_name, field_name); //this shouldn't be necessary, but sometimes helps
        SMScopedLock::create(m_interface_name, field_name);
      }

      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while creating new field_name \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getFloatingPointData(std::string field_name, unsigned long joint_idx, double& data)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>(field_name.c_str()).first;
    if(field_data_sm == 0) //no one has set the joint names yet
    {
      PRINT_TRACE_EXIT
      return false;
    }
    data = field_data_sm->at(joint_idx);
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::setFloatingPointData(std::string field_name, unsigned long joint_idx, double value)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>(field_name.c_str()).first;
    if(field_data_sm == 0) //no one has set the joint names yet
    {
      PRINT_TRACE_EXIT
      return false;
    }
    field_data_sm->at(joint_idx) = value;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getFloatingPointField(std::string field_name, std::vector<double>& field_data_local)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>(field_name.c_str()).first;
    if(field_data_sm == 0) //no one has set the joint names yet
    {
      PRINT_TRACE_EXIT
      return false;
    }

    field_data_local.resize(field_data_sm->size());
    for(unsigned int i = 0; i < field_data_sm->size(); i++)
    {
      field_data_local.at(i) = field_data_sm->at(i);
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::setFloatingPointField(std::string field_name, std::vector<double>& field_data_local)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>(field_name.c_str()).first;
    if(field_data_sm == 0) //field_name doesn't exist yet
    {
      PRINT_TRACE_EXIT
      return false;
    }

    if(field_data_local.size() != field_data_sm->size())
    {
      std::cerr << "SharedMemoryTransport: Tried to set field_name " << field_name << " of size " << field_data_sm->size() << " with data of size " << field_data_local.size() << "!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    for(unsigned int i = 0; i < field_data_sm->size(); i++)
    {
      field_data_sm->at(i) = field_data_local.at(i);
    }

    bool* invalid = segment.find<bool>((field_name + "_invalid").c_str()).first;
    *invalid = false;

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::checkFloatingPointField(std::string field_name)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    if(segment.find<SMDoubleVector>(field_name.c_str()).first == 0) //field_name doesn't exist
    {
      PRINT_TRACE_EXIT
      return false;
    }

    bool* invalid = segment.find<bool>((field_name + "_invalid").c_str()).first;
    if(*invalid)
    {
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getStringVectorField(std::string field_name, std::vector<std::string>& strings_local)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMStringVector* strings_sm = segment.find<SMStringVector>(field_name.c_str()).first;
    if(strings_sm == 0) //no one has set the joint strings yet
    {
      PRINT_TRACE_EXIT
      return false;
    }

    for(unsigned int i = 0; i < strings_sm->size(); i++)
    {
      std::string string = std::string(strings_sm->at(i).begin(), strings_sm->at(i).end());
      strings_local.push_back(string);
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::setStringVectorField(std::string field_name, std::vector<std::string>& strings_local)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock memory_lock(m_interface_name, "");
    SMScopedWriterLock lock(m_interface_name, field_name);

    unsigned long total_length = 0;
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      SMStringVector* strings_sm = segment.find<SMStringVector>(field_name.c_str()).first;
      if(strings_sm == 0) //no one has set the joint strings yet
      {
        PRINT_TRACE_EXIT
        return false;
      }

      if(strings_sm->size() != strings_local.size())
      {
        std::cerr << "SharedMemoryTransport: Tried to set string field_name " << field_name << " of size " << strings_sm->size() << " with data of size " << strings_local.size() << "!" << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }

      //figure out how much extra memory we need to store the strings
      for(unsigned int i = 0; i < strings_local.size(); i++)
      {
        total_length += strings_local.at(i).length();
      }
    }

    //populate the object
    try
    {
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), total_length * sizeof(char) + 4096); //add space for the new field_name's data + overhead

      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());

        const SMCharAllocator alloc_char_inst(segment.get_segment_manager());
        SMStringVector* strings_sm = segment.find<SMStringVector>(field_name.c_str()).first;
        strings_sm->clear();
        for(unsigned int i = 0; i < strings_local.size(); i++)
        {
          SMString string(alloc_char_inst);
          string = strings_local.at(i).c_str();
          strings_sm->push_back(string);
        }

        bool* invalid = segment.find<bool>((field_name + "_invalid").c_str()).first;
        *invalid = false;
      }

      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while setting string field_name \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::checkStringVectorField(std::string field_name)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    if(segment.find<SMStringVector>(field_name.c_str()).first == 0) //field_name doesn't exist
    {
      PRINT_TRACE_EXIT
      return false;
    }

    bool* invalid = segment.find<bool>((field_name + "_invalid").c_str()).first;
    if(*invalid)
    {
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getSerializedField(std::string field_name, std::string& data, std::string& md5sum, std::string& datatype)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMString* field_data_sm = segment.find<SMString>(field_name.c_str()).first;
    SMString* field_md5sum_sm = segment.find<SMString>((field_name + "_md5sum").c_str()).first;
    SMString* field_datatype_sm = segment.find<SMString>((field_name + "_datatype").c_str()).first;
    if(field_data_sm == 0) //field_name doesn't exist yet
    {
      std::cerr << "Couldn't find field_name " << field_name << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    data = std::string(field_data_sm->begin(), field_data_sm->end());
    md5sum = std::string(field_md5sum_sm->begin(), field_md5sum_sm->end());
    datatype = std::string(field_datatype_sm->begin(), field_datatype_sm->end());

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::setSerializedField(std::string field_name, std::string data, std::string md5sum, std::string datatype)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock memory_lock(m_interface_name, "");
    SMScopedWriterLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), data.length() * sizeof(char) + 4096); //add space for the new field_name's data + overhead
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      SMString* field_data_sm = segment.find<SMString>(field_name.c_str()).first;
      SMString* field_md5sum_sm = segment.find<SMString>((field_name + "_md5sum").c_str()).first;
      SMString* field_datatype_sm = segment.find<SMString>((field_name + "_datatype").c_str()).first;

      if(field_data_sm == 0) //field_name doesn't exist yet
      {
        std::cerr << "Couldn't find field_name " << field_name << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }
      std::string md5sum_sm = std::string(field_md5sum_sm->begin(), field_md5sum_sm->end());
      std::string datatype_sm = std::string(field_datatype_sm->begin(), field_datatype_sm->end());
      if(md5sum_sm != md5sum)
      {
        std::cerr << "SharedMemoryTransport: md5sum doesn't match for field_name " << field_name << "!" << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }
      if(datatype_sm != datatype)
      {
        std::cerr << "SharedMemoryTransport: datatype doesn't match for field_name " << field_name << "!" << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }

      *field_data_sm = SMString(data.begin(), data.end(), segment.get_segment_manager());

      bool* invalid = segment.find<bool>((field_name + "_invalid").c_str()).first;
      *invalid = false;
    }
    boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::checkSerializedField(std::string field_name)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    if(segment.find<SMString>(field_name.c_str()).first == 0) //field_name doesn't exist
    {
      PRINT_TRACE_EXIT
      return false;
    }

    bool* invalid = segment.find<bool>((field_name + "_invalid").c_str()).first;
    if(invalid == NULL)
    {
      std::cerr << "Companion fields for field " << field_name << " don't exist! Something is horribly wrong!" << std::endl;
      return false;
    }
    if(*invalid)
    {
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::hasConnections()
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock memory_lock(m_interface_name, "");

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    unsigned long* connection_tokens = segment.find<unsigned long>("connection_tokens").first;
    if(connection_tokens == 0)
    {
      std::cerr << "connection_tokens has not been created!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }
    PRINT_TRACE_EXIT
    return (*connection_tokens) > 1;
  }

  bool SharedMemoryTransport::hasNewData(std::string field_name)
  {
    PRINT_TRACE_ENTER
    SMScopedReaderLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;
    PRINT_TRACE_EXIT
    return (flag == 0)? false : *flag; //always false if the field_name doesn't exist
  }

  bool SharedMemoryTransport::awaitNewData(std::string field_name, double timeout)
  {
    PRINT_TRACE_ENTER
    //TODO: integrate condition variables into SMScopedLock?
    boost::interprocess::named_upgradable_mutex mutex(boost::interprocess::open_only, SMScopedLock::getFullName(m_interface_name, field_name).c_str());
    boost::interprocess::scoped_lock<boost::interprocess::named_upgradable_mutex> lock(mutex); //TODO: see if we can make this sharable
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());

    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;

    if((flag == 0)? false : *flag) //the data is already ready
    {
      PRINT_TRACE_EXIT
      return true;
    }

    if(timeout < 0)
    {
      boost::interprocess::named_condition(boost::interprocess::open_or_create, (field_name + "_ready").c_str(), unrestricted()).wait(lock);
    }
    else
    {
      boost::posix_time::ptime timeout_time = boost::get_system_time() + boost::posix_time::milliseconds(timeout);
      if(!boost::interprocess::named_condition(boost::interprocess::open_or_create, (field_name + "_ready").c_str(), unrestricted()).timed_wait(lock, timeout_time))
      {
        PRINT_TRACE_EXIT
        return false;
      }
    }
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::signalAvailable(std::string field_name)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;
    if(!flag)
    {
      PRINT_TRACE_EXIT
      return false;
    }

    boost::interprocess::named_condition(boost::interprocess::open_or_create, (field_name + "_ready").c_str(), unrestricted()).notify_all();

    *flag = true;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::signalProcessed(std::string field_name)
  {
    PRINT_TRACE_ENTER
    SMScopedWriterLock lock(m_interface_name, field_name);

    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;
    if(!flag)
    {
      PRINT_TRACE_EXIT
      return false;
    }

    *flag = false;
    PRINT_TRACE_EXIT
    return true;
  }
}
