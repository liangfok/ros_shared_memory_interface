/*
 * File: tutorial_sensor.cpp
 * Package: shared_memory_interface
 * Author: Joshua James
 * License: CC BY-SA 3.0 (attribution required)
 */

#include "shared_memory_interface/shared_memory_interface.hpp"
using namespace shared_memory_interface;

int main(int argc, char **argv)
{
  //initialize fake sensor data
  std::vector<double> position, velocity, acceleration, checkerboard;
  std::vector<std::string> joint_names;

  for(unsigned int i = 0; i < 10; i++)
  {
    position.push_back(i + 1);
    velocity.push_back((i + 1) * 10);
    acceleration.push_back((i + 1) * 100);

    std::stringstream ss;
    ss << "joint_" << i;
    joint_names.push_back(ss.str());
  }
  checkerboard.resize(10 * 10); //10x10 checkerboard "image"

  //construct interface
  SharedMemoryInterface smi("smi");
  smi.advertiseFPVector("position", 10);
  smi.advertiseFPVector("velocity", 10);
  smi.advertiseFPVector("acceleration", 10);
  smi.advertiseFPMatrix("checkerboard", 10, 10);
  smi.advertiseStringVector("joint_names", 10);

  //publish sensor data!
  double counter = 0.0;
  while(true)
  {
    for(unsigned int row = 0; row < 10; row++)
    {
      for(unsigned int col = 0; col < 10; col++)
      {
        unsigned long idx = row * 10 + col;
        bool black_square = (col % 2 == 0);
        if(row % 2 == 0)
        {
          black_square = !black_square;
        }
        double val = black_square? 0.0 : counter;
        checkerboard.at(idx) = val;
      }
    }

    smi.publishFPVector("position", position);
    smi.publishFPVector("velocity", velocity);
    smi.publishFPVector("acceleration", acceleration);
    smi.publishFPMatrix("checkerboard", checkerboard);
    smi.publishStringVector("joint_names", joint_names);

    std::cerr << "Sent sensor measurement " << counter << std::endl;

    counter += 1.0;
    if(counter > 1000.0)
    {
      counter = 0.0;
    }
    usleep(100000);//10hz
  }

  return 0;
}

