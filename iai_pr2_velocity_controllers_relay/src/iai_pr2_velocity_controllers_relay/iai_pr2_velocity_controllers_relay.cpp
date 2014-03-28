#include <ros/ros.h>

class Pr2VelocityControllersRelay
{

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_vel_ctrls_relay");

  ros::NodeHandle nh("~");
  Pr2VelocityControllersRelay my_relay;

  ros::spin();

  return 0;
}
