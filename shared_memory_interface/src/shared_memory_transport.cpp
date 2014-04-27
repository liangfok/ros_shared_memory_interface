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

  SharedMemoryTransport::SharedMemoryTransport(std::string interface_name)
  {
    PRINT_TRACE_ENTER
    std::cerr << "Initializing SharedMemoryTransport" << std::endl;
    m_interface_name = interface_name;
    m_mutex_name = interface_name + "_mutex";
    m_data_name = interface_name + "_data";
    m_mutex = new boost::interprocess::named_mutex(boost::interprocess::open_or_create, m_mutex_name.c_str());

    try
    {
      boost::posix_time::ptime timeout = boost::get_system_time() + boost::posix_time::milliseconds(1000);
      if(m_mutex->timed_lock(timeout))
      {
        m_mutex->unlock();
      }
      else //lock is most likely broken
      {
        std::cerr << "SharedMemoryTransport: SHARED MEMORY NOT CLEANED UP PROPERLY! DELETING OLD SPACES!\n" << " - m_mutex_name: " << m_mutex_name << "\n" << " - m_data_name: " << m_data_name << "\n";

        // delete old shared memory objects
        boost::interprocess::shared_memory_object::remove(m_data_name.c_str());
        boost::interprocess::named_mutex::remove(m_mutex_name.c_str());
        m_mutex = new boost::interprocess::named_mutex(boost::interprocess::open_or_create, m_mutex_name.c_str());
      }
    }
    catch(boost::interprocess::interprocess_exception ee)
    {
      std::cerr << "Exception while initializing shared memory interface!\n" << " - error: " << ee.what();
      throw ee;
    }

    {
      boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
      //try to open existing shared memory
      try
      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
        unsigned long* connection_tokens = segment.find<unsigned long>("connection_tokens").first;
        *connection_tokens = *connection_tokens + 1;
        std::cerr << "Connected to " << interface_name << " space." << std::endl;
      }
      catch(boost::interprocess::interprocess_exception &ex) //shared memory hasn't been created yet, so we'll make it
      {
        std::cerr << "SM space " << interface_name << " not found. Creating it." << std::endl;
        unsigned long base_size = 8192; //make sure we have enough room to create the default objects
        {
          boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::create_only, m_data_name.c_str(), base_size);

          //TODO: segregate rtt measurements
          segment.construct<unsigned char>("rtt_seq_no_tx")(0);
          segment.construct<unsigned char>("rtt_seq_no_rx")(0);
          unsigned long* connection_tokens = segment.construct<unsigned long>("connection_tokens")(0);
          *connection_tokens = *connection_tokens + 1;
        }
        boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str());
        std::cerr << "Created " << interface_name << " space." << std::endl;
      }

    }
    PRINT_TRACE_EXIT
  }

  SharedMemoryTransport::~SharedMemoryTransport()
  {
    {
      boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
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
    boost::interprocess::named_mutex::remove(m_mutex_name.c_str());
  }

  void SharedMemoryTransport::destroyMemory(std::string interface_name)
  {
    std::cerr << "Destroying shared memory space.";
    std::string mutex_name = interface_name + "_mutex";
    std::string data_name = interface_name + "_data";
    boost::interprocess::shared_memory_object::remove(data_name.c_str());
    boost::interprocess::named_mutex::remove(mutex_name.c_str());
  }

  bool SharedMemoryTransport::addFPMatrixField(std::string field_name, unsigned long rows, unsigned long cols)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);

    //check to see if someone else created the field
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      if(segment.find<SMDoubleVector>(field_name.c_str()).first)
      {
        //TODO:make sure it's the dimensions we think it should be
        std::cerr << "Found existing field " << field_name << std::endl;
        PRINT_TRACE_EXIT
        return true; //field already exists, so we're done
      }
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while testing existance of field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    //no one did, so we'll make it ourselves
    try
    {
      unsigned long size = rows * cols * sizeof(double) + 4096;
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), size); //add space for the new field's data + overhead
      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
        const SMDoubleAllocator alloc_double_inst(segment.get_segment_manager());
        SMDoubleVector* vector = segment.construct<SMDoubleVector>(field_name.c_str())(alloc_double_inst);
        vector->resize(rows * cols, 0.0);
        segment.construct<bool>(std::string(field_name + "_new_data_flag").c_str())(false);
        segment.construct<unsigned long>((field_name + "_row_stride").c_str())(cols); //row_stride = number of cols in each row
      }
      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while creating new field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::addSVField(std::string field_name, unsigned long length)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);

    //check to see if someone else created the field
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      if(segment.find<SMStringVector>(field_name.c_str()).first)
      {
        //TODO:make sure it's the dimensions we think it should be
        std::cerr << "Found existing field " << field_name << std::endl;
        PRINT_TRACE_EXIT
        return true; //field already exists, so we're done
      }
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while testing existance of field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    //no one did, so we'll make it ourselves
    try
    {
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), length * sizeof(char) + 4096); //add space for the new field's data + overhead

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
      }

      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while creating new field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::addSerializedField(std::string field_name, std::string md5sum, std::string datatype)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);

    //check to see if someone else created the field
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      if(segment.find<SMString>(field_name.c_str()).first)
      {
        std::cerr << "Found existing field " << field_name << std::endl;
        PRINT_TRACE_EXIT
        return true; //field already exists, so we're done
      }
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while testing existance of field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    //no one did, so we'll make it ourselves
    try
    {
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), 4096); //add space for the new field's data + overhead

      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());

        const SMCharAllocator alloc_char_inst(segment.get_segment_manager());
        SMString* string_sm = segment.construct<SMString>(field_name.c_str())(alloc_char_inst);
        *string_sm = "";

        SMString* md5sum_sm = segment.construct<SMString>((field_name + "_md5sum").c_str())(alloc_char_inst);
        SMString* datatype_sm = segment.construct<SMString>((field_name + "_datatype").c_str())(alloc_char_inst);
        *md5sum_sm = md5sum.c_str();
        *datatype_sm = datatype.c_str();

        segment.construct<bool>(std::string(field_name + "_new_data_flag").c_str())(false);
      }

      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while creating new field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getFPData(std::string field, unsigned long joint_idx, double& data)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>(field.c_str()).first;
    if(field_data_sm == 0) //no one has set the joint names yet
    {
      PRINT_TRACE_EXIT
      return false;
    }
    data = field_data_sm->at(joint_idx);
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::setFPData(std::string field, unsigned long joint_idx, double value)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>(field.c_str()).first;
    if(field_data_sm == 0) //no one has set the joint names yet
    {
      PRINT_TRACE_EXIT
      return false;
    }
    field_data_sm->at(joint_idx) = value;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getFPField(std::string field, std::vector<double>& field_data_local)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>(field.c_str()).first;
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

  bool SharedMemoryTransport::setFPField(std::string field, std::vector<double>& field_data_local)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>(field.c_str()).first;
    if(field_data_sm == 0) //field doesn't exist yet
    {
      PRINT_TRACE_EXIT
      return false;
    }

    if(field_data_local.size() != field_data_sm->size())
    {
      std::cerr << "SharedMemoryTransport: Tried to set field " << field << " of size " << field_data_sm->size() << " with data of size " << field_data_local.size() << "!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    for(unsigned int i = 0; i < field_data_sm->size(); i++)
    {
      field_data_sm->at(i) = field_data_local.at(i);
    }
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getSVField(std::string field_name, std::vector<std::string>& strings_local)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
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

  bool SharedMemoryTransport::setSVField(std::string field_name, std::vector<std::string>& strings_local)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);

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
        std::cerr << "SharedMemoryTransport: Tried to set string field " << field_name << " of size " << strings_sm->size() << " with data of size " << strings_local.size() << "!" << std::endl;
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
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), total_length * sizeof(char) + 4096); //add space for the new field's data + overhead

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
      }

      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryTransport: Exception " << ex.what() << " thrown while setting string field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getSerializedField(std::string field_name, std::string& data, std::string& md5sum, std::string& datatype)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMString* field_data_sm = segment.find<SMString>(field_name.c_str()).first;
    SMString* field_md5sum_sm = segment.find<SMString>((field_name + "_md5sum").c_str()).first;
    SMString* field_datatype_sm = segment.find<SMString>((field_name + "_datatype").c_str()).first;
    if(field_data_sm == 0) //field doesn't exist yet
    {
      std::cerr << "Couldn't find field " << field_name << std::endl;
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
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);

    boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), data.length() * sizeof(char) + 4096); //add space for the new field's data + overhead

    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      SMString* field_data_sm = segment.find<SMString>(field_name.c_str()).first;
      SMString* field_md5sum_sm = segment.find<SMString>((field_name + "_md5sum").c_str()).first;
      SMString* field_datatype_sm = segment.find<SMString>((field_name + "_datatype").c_str()).first;

      if(field_data_sm == 0) //field doesn't exist yet
      {
        std::cerr << "Couldn't find field " << field_name << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }
      std::string md5sum_sm = std::string(field_md5sum_sm->begin(), field_md5sum_sm->end());
      std::string datatype_sm = std::string(field_datatype_sm->begin(), field_datatype_sm->end());
      if(md5sum_sm != md5sum)
      {
        std::cerr << "SharedMemoryTransport: md5sum doesn't match for field " << field_name << "!" << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }
      if(datatype_sm != datatype)
      {
        std::cerr << "SharedMemoryTransport: datatype doesn't match for field " << field_name << "!" << std::endl;
        PRINT_TRACE_EXIT
        return false;
      }

      *field_data_sm = SMString(data.begin(), data.end(), segment.get_segment_manager());
    }

    boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::setTxSequenceNumber(unsigned char value)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    unsigned char* rtt_seq_no_tx = segment.find<unsigned char>("rtt_seq_no_tx").first;
    if(rtt_seq_no_tx == 0)
    {
      PRINT_TRACE_EXIT
      return false;
    }
    *rtt_seq_no_tx = value;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getTxSequenceNumber(unsigned char& sequence_number)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    unsigned char* rtt_seq_no_tx = segment.find<unsigned char>("rtt_seq_no_tx").first;
    if(rtt_seq_no_tx == 0)
    {
      PRINT_TRACE_EXIT
      return false;
    }
    sequence_number = *rtt_seq_no_tx;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::setRxSequenceNumber(unsigned char value)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    unsigned char* rtt_seq_no_rx = segment.find<unsigned char>("rtt_seq_no_rx").first;
    if(rtt_seq_no_rx == 0)
    {
      PRINT_TRACE_EXIT
      return false;
    }
    *rtt_seq_no_rx = value;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::getRxSequenceNumber(unsigned char& sequence_number)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    unsigned char* rtt_seq_no_rx = segment.find<unsigned char>("rtt_seq_no_rx").first;
    if(rtt_seq_no_rx == 0)
    {
      PRINT_TRACE_EXIT
      return false;
    }
    sequence_number = *rtt_seq_no_rx;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::hasConnections()
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
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
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;
    PRINT_TRACE_EXIT
    return (flag == 0)? false : *flag; //always false if the field doesn't exist
  }

  void SharedMemoryTransport::awaitNewData(std::string field_name)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());

    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;

    if((flag == 0)? false : *flag) //the data is already ready
    {
      return;
    }

    //TODO: destroy these conditions!
    boost::interprocess::named_condition(boost::interprocess::open_or_create, (field_name + "_ready").c_str()).wait(lock);
    PRINT_TRACE_EXIT
  }

  bool SharedMemoryTransport::signalAvailable(std::string field_name)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;
    if(!flag)
    {
      PRINT_TRACE_EXIT
      return false;
    }

    boost::interprocess::named_condition(boost::interprocess::open_or_create, (field_name + "_ready").c_str()).notify_all();

    *flag = true;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryTransport::signalProcessed(std::string field_name)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
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
