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

#include <unistd.h>
#include <boost/thread/mutex.hpp>
boost::mutex m_mutex;

std::vector<double> m_current_position;
std::vector<double> m_current_velocity;
std::vector<double> m_current_acceleration;
std::vector<double> m_current_checkerboard;
std::vector<std::string> m_names;

bool new_position, new_velocity, new_acceleration, new_checkerboard;

void printVector(std::string name, std::vector<double>& vector)
{
  std::cerr << "Got " << name << " vector: [";
  for(unsigned int i = 0; i < vector.size(); i++)
  {
    if(i == 0)
    {
      std::cerr << vector.at(i);
    }
    else
    {
      std::cerr << ", " << vector.at(i);
    }
  }
  std::cerr << "]" << std::endl;
}

void printVector(std::string name, std::vector<std::string>& vector)
{
  std::cerr << "Got " << name << " vector: [";
  for(unsigned int i = 0; i < vector.size(); i++)
  {
    if(i == 0)
    {
      std::cerr << vector.at(i);
    }
    else
    {
      std::cerr << ", " << vector.at(i);
    }
  }
  std::cerr << "]" << std::endl;
}

void positionCallback(std::vector<double>& msg)
{
  boost::mutex::scoped_lock lock(m_mutex);
  m_current_position = msg;
  printVector("position", msg);
  new_position = true;
}

void velocityCallback(std::vector<double>& msg)
{
  boost::mutex::scoped_lock lock(m_mutex);
  m_current_velocity = msg;
  printVector("velocity", msg);
  new_velocity = true;
}

void accelerationCallback(std::vector<double>& msg)
{
  boost::mutex::scoped_lock lock(m_mutex);
  m_current_acceleration = msg;
  printVector("acceleration", msg);
  new_acceleration = true;
}

void checkerboardCallback(std::vector<double>& msg)
{
  boost::mutex::scoped_lock lock(m_mutex);
  m_current_checkerboard = msg;
  printVector("checkerboard", msg);
  new_checkerboard = true;
}

void namesCallback(std::vector<std::string>& msg)
{
  boost::mutex::scoped_lock lock(m_mutex);
  m_names = msg;
  printVector("names", msg);
}

int main(int argc, char **argv)
{
  signal(SIGINT, loopBreaker);
  //construct interface
  SharedMemoryInterface smi("smi");

  new_position = false;
  new_velocity = false;
  new_acceleration = false;
  new_checkerboard = false;

  smi.waitForStringVector("joint_names", m_names); //joint names are usually static data, so use the one-off memory read instead of creating a subscriber
  printVector("names", m_names);
  //  smi.subscribeStringVector("joint_names", boost::bind(&namesCallback, _1));

  smi.subscribeFloatingPointVector("position", boost::bind(&positionCallback, _1));
  smi.subscribeFloatingPointVector("velocity", boost::bind(&velocityCallback, _1));
  smi.subscribeFloatingPointVector("acceleration", boost::bind(&accelerationCallback, _1));
  smi.subscribeFloatingPointVector("checkerboard", boost::bind(&checkerboardCallback, _1));

  smi.advertiseFloatingPointVector("command", 10);

  //process sensor data and send out commands
  ok = true;
  while(ok)
  {
    if(new_position && new_velocity && new_acceleration && new_checkerboard)
    {
      boost::mutex::scoped_lock lock(m_mutex);

      double current_count = m_current_checkerboard.at(0);

      std::vector<double> command;
      for(unsigned int i = 0; i < m_names.size(); i++)
      {
        command.push_back(current_count + (m_current_position.at(i) + m_current_velocity.at(i) + m_current_acceleration.at(i))/10000);
      }

      std::cerr << "Sending command\n";
      smi.publishFloatingPointVector("command", command);

      new_position = false;
      new_velocity = false;
      new_acceleration = false;
      new_checkerboard = false;
    }
    else
    {
//      std::cerr << "\n\n\nNo new sensor data! : " << new_position << " " << new_velocity << " " << new_acceleration << " " << new_checkerboard << " " << new_names;
    }
    usleep(100000);//10hz
  }

  return 0;
}

