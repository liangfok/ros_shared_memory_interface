#include "shared_memory_interface/shared_memory_interface.hpp"

namespace shared_memory_interface
{
#define TRACE 0
#define PRINT_TRACE_ENTER if(TRACE)std::cerr<<__func__<<std::endl;
#define PRINT_TRACE_EXIT if(TRACE)std::cerr<<"/"<<__func__<<std::endl;

  typedef boost::interprocess::allocator<double, boost::interprocess::managed_shared_memory::segment_manager> ShmemDoubleAllocator;
  typedef boost::interprocess::vector<double, ShmemDoubleAllocator> SMDoubleVector;

  typedef boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> ShmemCharAllocator;
  typedef boost::interprocess::basic_string<char, std::char_traits<char>, ShmemCharAllocator> ShmemString;
  typedef boost::interprocess::allocator<ShmemString, boost::interprocess::managed_shared_memory::segment_manager> ShmemStringAllocator;
  typedef boost::interprocess::vector<ShmemString, ShmemStringAllocator> SMStringVector;

  SharedMemoryInterface::SharedMemoryInterface(std::string interface_name)
  {
    PRINT_TRACE_ENTER
    std::cerr << "Initializing SMCI" << std::endl;
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
        std::cerr << "SharedMemoryInterface: SHARED MEMORY NOT CLEANED UP PROPERLY! DELETING OLD SPACES!\n" << " - m_mutex_name: " << m_mutex_name << "\n" << " - m_data_name: " << m_data_name << "\n";

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

  SharedMemoryInterface::~SharedMemoryInterface()
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
    std::cerr << "Deleting shared memory cleanly.\n";
    boost::interprocess::shared_memory_object::remove(m_data_name.c_str());
    boost::interprocess::named_mutex::remove(m_mutex_name.c_str());
  }

  void SharedMemoryInterface::destroyMemory(std::string interface_name)
  {
    std::cerr << "Destroying shared memory space.";
    std::string mutex_name = interface_name + "_mutex";
    std::string data_name = interface_name + "_data";
    boost::interprocess::shared_memory_object::remove(data_name.c_str());
    boost::interprocess::named_mutex::remove(mutex_name.c_str());
  }

  bool SharedMemoryInterface::addMatrixField(std::string field_name, unsigned long rows, unsigned long cols, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);

    //check to see if someone else created the field
    try
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      if(segment.find<SMDoubleVector>((sm_namespace + field_name).c_str()).first)
      {
        //TODO:make sure it's the dimensions we think it should be
        std::cerr << "Found existing field " << sm_namespace + field_name << std::endl;
        PRINT_TRACE_EXIT
        return true; //field already exists, so we're done
      }
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryInterface: Exception " << ex.what() << " thrown while testing existing of field \"" << field_name << "\"!" << std::endl;
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
        const ShmemDoubleAllocator alloc_double_inst(segment.get_segment_manager());
        SMDoubleVector* vector = segment.construct<SMDoubleVector>((sm_namespace + field_name).c_str())(alloc_double_inst);
        vector->resize(rows * cols, 0.0);
        segment.construct<bool>(std::string(sm_namespace + field_name + "_new_data_flag").c_str())(false);
        segment.construct<unsigned long>((sm_namespace + field_name + "_row_stride").c_str())(cols); //row_stride = number of cols in each row
      }
      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryInterface: Exception " << ex.what() << " thrown while creating new field \"" << field_name << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryInterface::addJointField(std::string field_name, unsigned long num_joints, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    bool success = addMatrixField(field_name, 1, num_joints, sm_namespace);
    PRINT_TRACE_EXIT
    return success;
//    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
//
//    //check to see if someone else created the field
//    try
//    {
//      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
//      if(segment.find<SMDoubleVector>((sm_namespace + field).c_str()).first)
//      {
//        //TODO:make sure it's the length we think it should be
//        return true; //field already exists, so we're done
//      }
//    }
//    catch(boost::interprocess::interprocess_exception &ex)
//    {
//      std::cerr << "SharedMemoryInterface: Exception " << ex.what() << " thrown while testing existing of field \"" << field_name << "\"!" << std::endl;
//      return false;
//    }
//
//    //no one did, so we'll make it ourselves
//    try
//    {
//      unsigned long size = num_joints * sizeof(double) + 4096;
//      std::cerr << "\n\nTrying to grow by " << size << "bytes...";
//      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), size); //add space for the new field's data + overhead
//      std::cerr << "Success!\n";
//      {
//        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
//        const ShmemDoubleAllocator alloc_double_inst(segment.get_segment_manager());
//        SMDoubleVector* vector = segment.construct<SMDoubleVector>((sm_namespace + field).c_str())(alloc_double_inst);
//        vector->resize(num_joints, 0.0);
//        segment.construct<bool>(std::string(field_name + "_new_data_flag").c_str())(false);
//      }
//      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
//    }
//    catch(boost::interprocess::interprocess_exception &ex)
//    {
//      std::cerr << "SharedMemoryInterface: Exception " << ex.what() << " thrown while creating new field \"" << field_name << "\"!" << std::endl;
//      return false;
//    }
//
//    return true;
  }

  bool SharedMemoryInterface::getData(std::string field, unsigned long joint_idx, double& data, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>((sm_namespace + field).c_str()).first;
    if(field_data_sm == 0) //no one has set the joint names yet
    {
      PRINT_TRACE_EXIT
      return false;
    }
    data = field_data_sm->at(joint_idx);
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryInterface::setData(std::string field, unsigned long joint_idx, double value, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>((sm_namespace + field).c_str()).first;
    if(field_data_sm == 0) //no one has set the joint names yet
    {
      PRINT_TRACE_EXIT
      return false;
    }
    field_data_sm->at(joint_idx) = value;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryInterface::getField(std::string field, std::vector<double>& field_data_local, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>((sm_namespace + field).c_str()).first;
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

  bool SharedMemoryInterface::setField(std::string field, std::vector<double>& field_data_local, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    SMDoubleVector* field_data_sm = segment.find<SMDoubleVector>((sm_namespace + field).c_str()).first;
    if(field_data_sm == 0) //field doesn't exist yet
    {
      PRINT_TRACE_EXIT
      return false;
    }

    if(field_data_local.size() != field_data_sm->size())
    {
      std::cerr << "SMCI: Tried to set field " << (sm_namespace + field) << " of size " << field_data_sm->size() << " with data of size " << field_data_local.size() << "!" << std::endl;
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

  bool SharedMemoryInterface::getJointNames(std::vector<std::string>& names_local, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    std::string full_name = sm_namespace + "names";
    SMStringVector* names_sm = segment.find<SMStringVector>(full_name.c_str()).first;
    std::cerr << "Getting joint names\n";
    if(names_sm == 0) //no one has set the joint names yet
    {
      std::cerr << "Joint names not found!";
      PRINT_TRACE_EXIT
      return false;
    }

    for(unsigned int i = 0; i < names_sm->size(); i++)
    {
      std::string name = std::string(names_sm->at(i).begin(), names_sm->at(i).end());
      names_local.push_back(name);
    }
    PRINT_TRACE_EXIT
    return true;
  }

  //TODO: make everything use full_name format
  bool SharedMemoryInterface::setJointNames(std::vector<std::string> names_local, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    std::string full_name = sm_namespace + "names";

    //assume that the names object either does not exist, or needs to be resized anyway, so destroy it if necessary and make it again
    //general use case is that this only gets called once per joint_set, so this isn't a problem
    {
      boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
      if(segment.find<SMStringVector>(full_name.c_str()).first)
      {
        segment.destroy<SMStringVector>(full_name.c_str());
      }
    }

    //figure out how much memory we need to store the strings
    unsigned long total_length = 0;
    for(unsigned int i = 0; i < names_local.size(); i++)
    {
      total_length += names_local.at(i).length();
    }

    //(re)make the object and populate it
    try
    {
      boost::interprocess::managed_shared_memory::grow(m_data_name.c_str(), total_length * sizeof(char) + 4096); //add space for the new field's data + overhead

      {
        boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());

        const ShmemCharAllocator alloc_char_inst(segment.get_segment_manager());
        SMStringVector* names_sm = segment.construct<SMStringVector>(full_name.c_str())(segment.get_segment_manager());
        for(unsigned int i = 0; i < names_local.size(); i++)
        {
          ShmemString name(alloc_char_inst);
          name = names_local.at(i).c_str();
          names_sm->push_back(name);
        }
      }
      boost::interprocess::managed_shared_memory::shrink_to_fit(m_data_name.c_str()); //don't overuse memory
    }
    catch(boost::interprocess::interprocess_exception &ex)
    {
      std::cerr << "SharedMemoryInterface: Exception " << ex.what() << " thrown while setting names for joint namespace \"" << sm_namespace << "\"!" << std::endl;
      PRINT_TRACE_EXIT
      return false;
    }

    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryInterface::setTxSequenceNumber(unsigned char value)
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

  bool SharedMemoryInterface::getTxSequenceNumber(unsigned char& sequence_number)
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

  bool SharedMemoryInterface::setRxSequenceNumber(unsigned char value)
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

  bool SharedMemoryInterface::getRxSequenceNumber(unsigned char& sequence_number)
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

  bool SharedMemoryInterface::hasConnections()
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

  bool SharedMemoryInterface::hasNew(std::string field_name, std::string sm_namespace)
  {
    PRINT_TRACE_ENTER
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_mutex);
    boost::interprocess::managed_shared_memory segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, m_data_name.c_str());
    bool* flag = segment.find<bool>(std::string(field_name + "_new_data_flag").c_str()).first;
    PRINT_TRACE_EXIT
    return (flag == 0)? false : *flag; //always false if the field doesn't exist
  }

  bool SharedMemoryInterface::signalAvailable(std::string field_name, std::string sm_namespace)
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

    *flag = true;
    PRINT_TRACE_EXIT
    return true;
  }

  bool SharedMemoryInterface::signalProcessed(std::string field_name, std::string sm_namespace)
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
