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
  smi.advertiseFloatingPointVector("position", 10);
  smi.advertiseFloatingPointVector("velocity", 10);
  smi.advertiseFloatingPointVector("acceleration", 10);
  smi.advertiseFloatingPointMatrix("checkerboard", 10, 10);
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

    smi.publishFloatingPointVector("position", position);
    smi.publishFloatingPointVector("velocity", velocity);
    smi.publishFloatingPointVector("acceleration", acceleration);
    smi.publishFloatingPointMatrix("checkerboard", checkerboard);
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

