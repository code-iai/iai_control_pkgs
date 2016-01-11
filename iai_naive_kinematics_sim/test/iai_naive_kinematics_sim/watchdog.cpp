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

#include <gtest/gtest.h>
#include <iai_naive_kinematics_sim/iai_naive_kinematics_sim.hpp>

class WatchdogTest : public ::testing::Test
{
   protected:
    virtual void SetUp(){}
    virtual void TearDown(){}

    void checkWatchdogContent(const iai_naive_kinematics_sim::Watchdog& dog, 
        double period, double update_time, double command)
    {
      EXPECT_EQ(ros::Duration(period), dog.getPeriod()); 
      EXPECT_EQ(ros::Time(update_time), dog.getLastUpdateTime()); 
      EXPECT_DOUBLE_EQ(command, dog.getCommand());
    }
};

TEST_F(WatchdogTest, SaneConstructor)
{
  iai_naive_kinematics_sim::Watchdog dog;
  checkWatchdogContent(dog, 0.0, 0.0, 0.0);
}

TEST_F(WatchdogTest, AlternativeConstructor)
{
  iai_naive_kinematics_sim::Watchdog dog(ros::Duration(1.1));
  checkWatchdogContent(dog, 1.1, 0.0, 0.0);
}
TEST_F(WatchdogTest, BarkTest)
{
  iai_naive_kinematics_sim::Watchdog dog;
  dog.setPeriod(ros::Duration(0.1));

  dog.update(ros::Time(0.0));
  checkWatchdogContent(dog, 0.1, 0.0, 0.0);
  EXPECT_EQ(ros::Duration(0.1), dog.getPeriod());
  EXPECT_EQ(ros::Time(0.0), dog.getLastUpdateTime());
  EXPECT_DOUBLE_EQ(0.0, dog.getCommand());

  dog.setNewCommand(ros::Time(0.03), 1.0);
  checkWatchdogContent(dog, 0.1, 0.03, 1.0);

  dog.update(ros::Time(0.06));
  checkWatchdogContent(dog, 0.1, 0.03, 1.0);

  dog.update(ros::Time(0.131));
  checkWatchdogContent(dog, 0.1, 0.03, 0.0);
}
