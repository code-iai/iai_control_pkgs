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

class SystemTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      controlled_joints_.push_back("joint2");
      model_.initFile("test_robot.urdf");

      iai_naive_kinematics_sim::pushBackJointState(state1_, "joint1", 0.0, 0.0, 0.0);
      iai_naive_kinematics_sim::pushBackJointState(state1_, "joint2", 0.0, 0.0, 0.0);

      iai_naive_kinematics_sim::pushBackJointState(state2_, "joint1", 0.0, 0.0, 0.0);
      iai_naive_kinematics_sim::pushBackJointState(state2_, "joint2", 0.0025, 0.05, 0.0);
      state2_.header.stamp = ros::Time(0.1);
      state2_.header.seq = 1;

      iai_naive_kinematics_sim::pushBackJointState(state3_, "joint1", 0.1, 0.2, 0.3);
      iai_naive_kinematics_sim::pushBackJointState(state3_, "joint2", -0.001, -0.002, -0.003);
     }

    virtual void TearDown(){}

    void checkJointStatesEquality(const sensor_msgs::JointState& a, const sensor_msgs::JointState& b) const
    {
      EXPECT_EQ(a.header.seq, b.header.seq);
      EXPECT_EQ(a.header.stamp, b.header.stamp);
      EXPECT_STREQ(a.header.frame_id.c_str(), b.header.frame_id.c_str());

      EXPECT_EQ(a.name.size(), b.name.size());
      EXPECT_EQ(a.position.size(), b.position.size());
      EXPECT_EQ(a.velocity.size(), b.velocity.size());
      EXPECT_EQ(a.effort.size(), b.effort.size());
    
      ASSERT_EQ(a.name.size(), a.position.size());
      ASSERT_EQ(a.name.size(), a.velocity.size());
      ASSERT_EQ(a.name.size(), a.effort.size());
 
      for(size_t i=0; i<a.name.size(); ++i)
      {
        EXPECT_STREQ(a.name[i].c_str(), b.name[i].c_str());
        EXPECT_DOUBLE_EQ(a.position[i], b.position[i]);
        EXPECT_DOUBLE_EQ(a.velocity[i], b.velocity[i]);
        EXPECT_DOUBLE_EQ(a.effort[i], b.effort[i]);
      }
    }

    std::vector<std::string> controlled_joints_;
    urdf::Model model_;
    sensor_msgs::JointState state1_, state2_, state3_;
};

TEST_F(SystemTest, SaneConstructor)
{
  iai_naive_kinematics_sim::System sys;

  EXPECT_TRUE(sys.getWatchdogs().empty());
  checkJointStatesEquality(sys.getJointState(), sensor_msgs::JointState());
}

TEST_F(SystemTest, Init)
{
  iai_naive_kinematics_sim::System sys;
  ASSERT_NO_THROW(sys.init(model_, controlled_joints_, 0.1));

  checkJointStatesEquality(state1_, sys.getJointState());
  ASSERT_EQ(1, sys.getWatchdogs().size());

  EXPECT_STREQ("joint2", sys.getWatchdogs().begin()->first.c_str());
  EXPECT_EQ(ros::Duration(0.1), sys.getWatchdogs().begin()->second.getPeriod());
  EXPECT_DOUBLE_EQ(0.0, sys.getWatchdogs().begin()->second.getCommand());
  EXPECT_EQ(ros::Time(0.0), sys.getWatchdogs().begin()->second.getLastUpdateTime());
}

TEST_F(SystemTest, SetVelocityCommand)
{
  iai_naive_kinematics_sim::System sys;
  ASSERT_NO_THROW(sys.init(model_, controlled_joints_, 0.1));
  ASSERT_NO_THROW(sys.setVelocityCommand("joint2", 0.05, ros::Time(0.05)));
  
  checkJointStatesEquality(state1_, sys.getJointState());

  ASSERT_NO_THROW(sys.update(ros::Time(0.1), 0.05));
  checkJointStatesEquality(state2_, sys.getJointState());
}

TEST_F(SystemTest, SetSubJointState)
{
  iai_naive_kinematics_sim::System sys;
  ASSERT_NO_THROW(sys.init(model_, controlled_joints_, 0.1));
  ASSERT_NO_THROW(sys.setSubJointState(state3_));

  checkJointStatesEquality(state3_, sys.getJointState());
}
