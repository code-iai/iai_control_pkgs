#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JointStateRestamper
{
  public:
    JointStateRestamper(const ros::NodeHandle& nh): nh_(nh)
    {
      publisher_ = nh_.advertise<sensor_msgs::JointState>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &JointStateRestamper::callback, this);
    }

    ~JointStateRestamper() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    void callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      publisher_.publish(restamp(*msg));
    }

    sensor_msgs::JointState restamp(const sensor_msgs::JointState& in_msg)
    {
      sensor_msgs::JointState out_msg(in_msg);
      out_msg.header.stamp = ros::Time::now(); 
      return out_msg;
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_state_stamper");

  ros::NodeHandle nh("~");

  JointStateRestamper joint_state_stamper(nh);

  ros::spin();

  return 0;
}
