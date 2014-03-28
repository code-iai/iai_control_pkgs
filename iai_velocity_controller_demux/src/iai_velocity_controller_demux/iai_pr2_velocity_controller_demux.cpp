#include <ros/ros.h>

class Pr2VelocityControllerDemux
{

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_velocity_controller_demux");

  ros::NodeHandle nh("~");
  Pr2VelocityControllerDemux my_demux;

  ros::spin();

  return 0;
}
