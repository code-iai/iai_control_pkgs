#include <ros/ros.h>
#include <pr2_msgs/PressureState.h>

class Pr2FingertipRestamper
{
  public:
    Pr2FingertipRestamper(const ros::NodeHandle& nh): nh_(nh)
    {
      publisher_ = nh_.advertise<pr2_msgs::PressureState>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &Pr2FingertipRestamper::callback, this);
    }

    ~Pr2FingertipRestamper() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    void callback(const pr2_msgs::PressureState::ConstPtr& msg)
    {
      publisher_.publish(restamp(*msg));
    }

    pr2_msgs::PressureState restamp(const pr2_msgs::PressureState& in_msg)
    {
      pr2_msgs::PressureState out_msg(in_msg);
      out_msg.header.stamp = ros::Time::now(); 
      return out_msg;
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_fingertip_restamper");

  ros::NodeHandle nh("~");

  Pr2FingertipRestamper pr2_fingertip_restamper(nh);

  ros::spin();

  return 0;
}
