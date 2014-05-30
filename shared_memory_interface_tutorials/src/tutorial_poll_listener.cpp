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
#include <signal.h>
using namespace shared_memory_interface;

bool ok;
void loopBreaker(int sig)
{
  ok = false;
}

int main(int argc, char **argv)
{
  signal(SIGINT, loopBreaker);
  SharedMemoryInterface smi("smi");

  ok = true;
  while(ok)
  {
    std::vector<double> msg;
    smi.getCurrentFloatingPointVector("chatter", msg);

//    std::cerr << "Setting actuator effort to [";
//    for(unsigned int i = 0; i < msg.size(); i++)
//    {
//      if(i == 0)
//      {
//        std::cerr << msg.at(i);
//      }
//      else
//      {
//        std::cerr << ", " << msg.at(i);
//      }
//    }
//    std::cerr << "]" << std::endl;

    usleep(100000);//10hz
  }

  return 0;
}

