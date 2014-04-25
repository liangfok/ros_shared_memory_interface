#ifndef SM_WATCHDOG_H
#define SM_WATCHDOG_H
#include <ros/ros.h>
#include "shared_memory_interface/shared_memory_interface.hpp"
#include <signal.h>

namespace shared_memory_interface
{
  class SMWatchdog
  {
  public:
    SMWatchdog(const ros::NodeHandle& nh);
    ~SMWatchdog();
    void spin();

  private:
    ros::NodeHandle m_nh;
    double m_loop_rate;
  };
}
#endif //SM_WATCHDOG_H
