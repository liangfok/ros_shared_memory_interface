/*
 * File: tutorial_actuator.cpp
 * Package: shared_memory_interface
 * Author: Joshua James
 * License: CC BY-SA 3.0 (attribution required)
 */

#include "shared_memory_interface/shared_memory_interface.hpp"
using namespace shared_memory_interface;

void commandCallback(std::vector<double>& msg)
{
  std::cerr << "Setting actuator effort to [";
  for(unsigned int i = 0; i < msg.size(); i++)
  {
    if(i == 0)
    {
      std::cerr << msg.at(i);
    }
    else
    {
      std::cerr << ", " << msg.at(i);
    }
  }
  std::cerr << "]" << std::endl;
}

int main(int argc, char **argv)
{
  SharedMemoryInterface smi("smi");
  smi.subscribeFPVector("command", boost::bind(&commandCallback, _1));

  while(true)
  {
    usleep(100000);//10hz
  }

  return 0;
}

