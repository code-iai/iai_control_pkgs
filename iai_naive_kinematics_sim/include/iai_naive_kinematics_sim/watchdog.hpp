/*
 * Copyright (c) 2015, Georg Bartels, <georg.bartels@cs.uni-bremen.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Institute of Artificial Intelligence, 
 *     University of Bremen nor the names of its contributors may be used 
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
