#include "shared_memory_interface/shared_memory_interface.hpp"
using namespace shared_memory_interface;
#include <unistd.h>
#include <boost/thread/mutex.hpp>
boost::mutex m_mutex;

std::vector<double> m_current_position;
std::vector<double> m_current_velocity;
std::vector<double> m_current_acceleration;
std::vector<double> m_current_checkerboard;
std::vector<std::string> m_names;

bool new_position, new_velocity, new_acceleration, new_checkerboard, new_names;

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
  new_names = true;
}

int main(int argc, char **argv)
{
  //construct interface
  SharedMemoryInterface smi("smi");

  new_position = false;
  new_velocity = false;
  new_acceleration = false;
  new_checkerboard = false;
  new_names = false;

  smi.subscribeFPVector("position", boost::bind(&positionCallback, _1));
  smi.subscribeFPVector("velocity", boost::bind(&velocityCallback, _1));
  smi.subscribeFPVector("acceleration", boost::bind(&accelerationCallback, _1));
  smi.subscribeFPVector("checkerboard", boost::bind(&checkerboardCallback, _1));
  smi.subscribeStringVector("joint_names", boost::bind(&namesCallback, _1));

  smi.advertiseFPVector("command", 10);

  //process sensor data and send out commands
  while(true)
  {
    if(new_position && new_velocity && new_acceleration && new_checkerboard && new_names)
    {
      std::cerr << "\n\n\nProcessing sensor data\n\n\n";
      boost::mutex::scoped_lock lock(m_mutex);

      double current_count = m_current_checkerboard.at(0);

      std::vector<double> command;
      for(unsigned int i = 0; i < m_names.size(); i++)
      {
        command.push_back(current_count + m_current_position.at(i) + m_current_velocity.at(i) + m_current_acceleration.at(i));
      }

      std::cerr << "Sending command\n";
      smi.publishFPVector("command", command);

      new_position = false;
      new_velocity = false;
      new_acceleration = false;
      new_checkerboard = false;
      new_names = false;
    }
    else
    {
      std::cerr << "\n\n\nNo new sensor data! : " << new_position << " " << new_velocity << " " << new_acceleration << " " << new_checkerboard << " " << new_names;
    }
    usleep(100000);//10hz
  }

  return 0;
}

