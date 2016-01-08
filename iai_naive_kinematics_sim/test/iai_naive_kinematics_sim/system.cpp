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
    sensor_msgs::JointState state1_, state2_;
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
