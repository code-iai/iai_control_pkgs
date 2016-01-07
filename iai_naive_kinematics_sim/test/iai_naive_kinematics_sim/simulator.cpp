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
