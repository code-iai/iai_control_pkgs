#include <gtest/gtest.h>
#include <iai_naive_kinematics_sim/simulator.hpp>

class SimulatorTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      joint_names_.clear();
      joint_names_.push_back("joint0");
      joint_names_.push_back("joint1");

      dt_ = 0.5;
      now_ = ros::Time(1.1);

      state_.name = joint_names_;
      state_.position.push_back(0.1);
      state_.position.push_back(0.2);
      state_.velocity.push_back(1.0);
      state_.velocity.push_back(2.0);
      state_.effort.push_back(0.0);
      state_.effort.push_back(0.0);

      state2_.name = joint_names_;
      state2_.position.push_back(state_.position[0] + state_.velocity[0] * dt_);
      state2_.position.push_back(state_.position[1] + state_.velocity[1] * dt_);
      state2_.velocity.push_back(state_.velocity[0]);
      state2_.velocity.push_back(state_.velocity[1]);
      state2_.effort.push_back(0.0);
      state2_.effort.push_back(0.0);
      state2_.header.stamp = now_;
      state2_.header.seq = 1;

      state3_.name = joint_names_;
      state3_.position.push_back(0.0);
      state3_.position.push_back(0.0);
      state3_.velocity.push_back(0.0);
      state3_.velocity.push_back(1.3);
      state3_.effort.push_back(0.0);
      state3_.effort.push_back(0.0);

      state4_.name = joint_names_;
      state4_.position.push_back(0.0);
      state4_.position.push_back(0.0);
      state4_.velocity.push_back(1.1);
      state4_.velocity.push_back(1.3);
      state4_.effort.push_back(0.0);
      state4_.effort.push_back(0.0);

      zero_state_ = makeZeroJointState(joint_names_);
    }

    virtual void TearDown(){}

    std::vector<std::string> joint_names_;
    sensor_msgs::JointState state_, state2_, state3_, state4_, zero_state_;
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

    sensor_msgs::JointState makeZeroJointState(const std::vector<std::string>& joint_names) const
    {
      sensor_msgs::JointState result;
      result.name = joint_names;

      for(size_t i=0; i<joint_names.size(); ++i)
      {
        result.position.push_back(0.0);
        result.velocity.push_back(0.0);
        result.effort.push_back(0.0);
      }

      return result;
    }
};

TEST_F(SimulatorTest, SaneConstructor)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  EXPECT_EQ(0, sim.size());

  sensor_msgs::JointState state = sim.getJointState();
  checkJointStatesEquality(sim.getJointState(), sensor_msgs::JointState());
}

TEST_F(SimulatorTest, InitFromJointNames)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  sim.init(joint_names_);

  EXPECT_EQ(joint_names_.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), zero_state_);
}

TEST_F(SimulatorTest, InitFromJointState)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  sim.init(state_);

  EXPECT_EQ(state_.name.size(), sim.size());
  checkJointStatesEquality(sim.getJointState(), state_);
}

TEST_F(SimulatorTest, Update)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  sim.init(state_);
  
  ASSERT_NO_THROW(sim.update(now_, dt_));
  checkJointStatesEquality(sim.getJointState(), state2_);
}

TEST_F(SimulatorTest, SetNextJointVelocity)
{
  iai_naive_kinematics_sim::SimulatorVelocityResolved sim;
  sim.init(joint_names_);

  ASSERT_NO_THROW(sim.setNextJointVelocity("joint1", 1.3));
  checkJointStatesEquality(sim.getJointState(), state3_);
 
  ASSERT_NO_THROW(sim.setNextJointVelocity("joint0", 1.1));
  checkJointStatesEquality(sim.getJointState(), state4_);
}
