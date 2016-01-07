#ifndef IAI_NAIVE_KINEMATICS_SIM_WATCHDOG_HPP
#define IAI_NAIVE_KINEMATICS_SIM_WATCHDOG_HPP

#include <ros/ros.h>

namespace iai_naive_kinematics_sim
{
  class Watchdog
  {
    public:
      Watchdog() : command_(0.0), period_(ros::Duration(0.0)), last_update_(ros::Time(0.0)) {}
      Watchdog(const ros::Duration& period) : command_(0.0), period_(period), last_update_(ros::Time(0.0)) {}
      ~Watchdog() {}

      const ros::Duration& getPeriod() const
      {
        return period_;
      }

      void setPeriod(const ros::Duration& period)
      {
        period_ = period;
      }
      double getCommand() const
      {
        return command_;
      }

      void setNewCommand(const ros::Time& now, double command)
      {
        command_ = command;
        last_update_ = now;
        update(now);
      }

      const ros::Time& getLastUpdateTime() const
      {
        return last_update_;
      }

      void update(const ros::Time& now)
      {
        if((now-last_update_) > period_)
          command_ = 0.0;
      }

    private:
      double command_;
      ros::Duration period_;
      ros::Time last_update_;
  };
}

#endif
