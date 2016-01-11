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

class SimulatorTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      dt_ = 0.5;
      now_ = ros::Time(1.1);

      iai_naive_kinematics_sim::pushBackJointState(zero_state_, "joint1", 0.0, 0.0, 0.0);
      iai_naive_kinematics_sim::pushBackJointState(zero_state_, "joint2", 0.0, 0.0, 0.0);

      iai_naive_kinematics_sim::pushBackJointState(state1_, "joint1", 1.1, 1.2, 1.3);
      iai_naive_kinematics_sim::pushBackJointState(state1_, "joint2", -0.01, -0.02, -0.03);

      iai_naive_kinematics_sim::pushBackJointState(state2_, "joint1", 0.0, 0.0, 0.0);
      iai_naive_kinematics_sim::pushBackJointState(state2_, "joint2", 0.1, 0.2, 0.3);

      iai_naive_kinematics_sim::pushBackJointState(sub_state2_, "joint2", 0.1, 0.2, 0.3);

      iai_naive_kinematics_sim::pushBackJointState(state3_, "joint1", 1.7, 1.2, 1.3);
      iai_naive_kinematics_sim::pushBackJointState(state3_, "joint2", -0.02, -0.02, -0.03);
      state3_.header.seq = 1;
      state3_.header.stamp = now_;

      iai_naive_kinematics_sim::pushBackJointState(state4_, "joint1", 0.0, 0.0, 0.0);
      iai_naive_kinematics_sim::pushBackJointState(state4_, "joint2", 0.0, 7.75, 0.0);

      iai_naive_kinematics_sim::pushBackJointState(state5_, "joint1", 3.007, 0.0, 1.3);
      iai_naive_kinematics_sim::pushBackJointState(state5_, "joint2", -0.05, -0.02, -0.03);
      state5_.header.seq = 1;
      state5_.header.stamp = now_;

      iai_naive_kinematics_sim::pushBackJointState(state6_, "joint1", 3.007, 0.0, 1.3);
      iai_naive_kinematics_sim::pushBackJointState(state6_, "joint2", -0.1, 0.0, -0.03);
      state6_.header.seq = 2;
      state6_.header.stamp = now_;

      model_.initFile("test_robot.urdf");
    }

    virtual void TearDown(){}

    urdf::Model model_;
    sensor_msgs::JointState state1_, sub_state2_, state2_, state3_, state4_, 
      state5_, state6_, zero_state_;
    double dt_;
    ros::Time now_;

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
};

TEST_F(SimulatorTest, SaneConstructor)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;

  EXPECT_EQ(0, sim.size());
  checkJointStatesEquality(sim.getJointState(), sensor_msgs::JointState());
}

TEST_F(SimulatorTest, Init)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  ASSERT_NO_THROW(sim.init(model_));

  EXPECT_EQ(zero_state_.name.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), zero_state_);
}

TEST_F(SimulatorTest, setSubJointState1)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  ASSERT_NO_THROW(sim.init(model_));
  ASSERT_NO_THROW(sim.setSubJointState(state1_));

  EXPECT_EQ(state1_.name.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), state1_);
}

TEST_F(SimulatorTest, setSubJointState2)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  ASSERT_NO_THROW(sim.init(model_));
  ASSERT_NO_THROW(sim.setSubJointState(sub_state2_));

  EXPECT_EQ(state1_.name.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), state2_);
}

TEST_F(SimulatorTest, Update)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  ASSERT_NO_THROW(sim.init(model_));
  ASSERT_NO_THROW(sim.setSubJointState(state1_));
  ASSERT_NO_THROW(sim.update(now_, dt_));

  EXPECT_EQ(state1_.name.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), state3_);
}

TEST_F(SimulatorTest, SetNextJointVelocity)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  ASSERT_NO_THROW(sim.init(model_));
  ASSERT_NO_THROW(sim.setNextJointVelocity("joint2", 7.75));

  EXPECT_EQ(state1_.name.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), state4_);
}

TEST_F(SimulatorTest, JointPositionLimits)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  ASSERT_NO_THROW(sim.init(model_));
  ASSERT_NO_THROW(sim.setSubJointState(state1_));
  ASSERT_NO_THROW(sim.update(now_, 4*dt_));

  EXPECT_EQ(state1_.name.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), state5_);

  ASSERT_NO_THROW(sim.update(now_, 6*dt_));

  EXPECT_EQ(state1_.name.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), state6_);
}

TEST_F(SimulatorTest, HasJoint)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  ASSERT_NO_THROW(sim.init(model_));

  EXPECT_FALSE(sim.hasJoint("joint0"));
  EXPECT_TRUE(sim.hasJoint("joint1"));
  EXPECT_TRUE(sim.hasJoint("joint2"));
  EXPECT_FALSE(sim.hasJoint("joint3"));
}
