#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

//
// IDEA BEHIND THIS NODE:
// The data published by the force/torque sensor of the PR2
// does not contain a frame-id in its header. As a result,
// the measured wrench cannot be displayed in rviz. 
//
// This node offers functionality to add a frame-id to those
// measurements and publish them on a new topic. 
//
// Node parameters:
//   '~frame_id': a string to put into the header of the wrench
//
// Node subscriptions:
//   '~in_topic': geometry_msgs/WrenchStamped coming from robot
//
// Node publications:
//   '~out_topic': geometry_msgs/WrenchStamped with frame-id
//
 
class WrenchStamper
{
  public:
    WrenchStamper(const ros::NodeHandle& nh): nh_(nh)
    {
      if(!nh_.getParam("frame_id", frame_id_))
        ROS_ERROR("[WrenchStamper]: Did not find parameter 'frame_id'.");

      publisher_ = nh_.advertise<geometry_msgs::WrenchStamped>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &WrenchStamper::callback, this);
    }

    ~WrenchStamper() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::string frame_id_;

    geometry_msgs::WrenchStamped add_frame_id(
        const geometry_msgs::WrenchStamped& wrench_in,
        const std::string& frame_id)
    {
      geometry_msgs::WrenchStamped wrench_out(wrench_in);
      wrench_out.header.frame_id = frame_id;
      return wrench_out;
    }

    void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
      publisher_.publish(add_frame_id(*msg, frame_id_));
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wrench_stamper");

  ros::NodeHandle nh("~");

  WrenchStamper wrench_stamper(nh);

  ros::spin();

  return 0;
}
