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
