#ifndef SHARED_MEMORY_INTERFACE_HPP
#define SHARED_MEMORY_INTERFACE_HPP

#include "shared_memory_interface/shared_memory_transport.hpp"

#include <vector>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/type_traits.hpp>

namespace shared_memory_interface
{
  class SharedMemoryInterface
  {
    //high efficiency implementations, limited data types
    //use SharedMemoryInterfaceROS if you want more flexibility at a possible cost of performance
  public:
    SharedMemoryInterface(std::string interface_name);
    ~SharedMemoryInterface();

    static void destroyMemory(std::string interface_name);

    //advertise the existance of a field in shared memory
    bool advertiseStringVector(std::string field_name, unsigned long length);
    bool advertiseFPVector(std::string field_name, unsigned long length);
    bool advertiseFPMatrix(std::string field_name, unsigned long rows, unsigned long cols);

    //publish data to the field and notify other processes
    bool publishStringVector(std::string field, std::vector<std::string>& data);
    bool publishFPVector(std::string field, std::vector<double>& data);
    bool publishFPMatrix(std::string field, std::vector<double>& data); //TODO: add implementation that takes an actual matrix type

    //set up a "subscriber" that calls a callback function whenever data is published by another process
    bool subscribeStringVector(std::string field, boost::function<void(std::vector<std::string>&)> callback);
    bool subscribeFPVector(std::string field, boost::function<void(std::vector<double>&)> callback);
    bool subscribeFPMatrix(std::string field, boost::function<void(std::vector<double>&)> callback);

    //these methods block until data is available or the timeout is exceeded, then return the data in the data field
    //useful for static data that only needs to be read once (no need to set up a subscriber)
    bool waitForStringVector(std::string field, std::vector<std::string>& data, double timeout = -1);
    bool waitForFPVector(std::string field, std::vector<double>& data, double timeout = -1);
    bool waitForFPMatrix(std::string field, std::vector<double>& data, double timeout = -1);

  protected:
    SharedMemoryTransport m_smt;
    std::string m_interface_name;

    std::vector<boost::thread*> m_callback_threads;
    bool m_shutdown;

    void callbackThreadFunctionFP(std::string field_name, boost::function<void(std::vector<double>&)> callback);
    void callbackThreadFunctionSV(std::string field_name, boost::function<void(std::vector<std::string>&)> callback);
  };

}
#endif //SHARED_MEMORY_INTERFACE_HPP
