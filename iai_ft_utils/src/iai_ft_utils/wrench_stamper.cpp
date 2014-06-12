#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

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
