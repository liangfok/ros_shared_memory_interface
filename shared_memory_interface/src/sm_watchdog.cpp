#include "shared_memory_interface/sm_watchdog.hpp"

void destroySharedMemory(int param)
{
  shared_memory_interface::SharedMemoryInterface::destroyMemory("smci");
  ros::shutdown();
}

namespace shared_memory_interface
{
  SMWatchdog::SMWatchdog(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_loop_rate, 10.0);
  }

  SMWatchdog::~SMWatchdog()
  {
    destroySharedMemory(0);
  }

  void SMWatchdog::spin()
  {
    ROS_INFO("SMWatchdog started.");
    ros::Rate loop_rate(m_loop_rate);
    while(ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_node_template");
  ros::NodeHandle nh("~");

  signal(SIGABRT, destroySharedMemory);
  signal(SIGFPE, destroySharedMemory);
  signal(SIGILL, destroySharedMemory);
  signal(SIGINT, destroySharedMemory);
  signal(SIGSEGV, destroySharedMemory);
  signal(SIGTERM, destroySharedMemory);

  shared_memory_interface::SMWatchdog node(nh);
  node.spin();

  return 0;
}
