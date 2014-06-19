#include <ros/ros.h>
#include <saphari_msgs/Human.h>

class HumanRestamper
{
  public:
    HumanRestamper(const ros::NodeHandle& nh): nh_(nh)
    {
      publisher_ = nh_.advertise<saphari_msgs::Human>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &HumanRestamper::callback, this);
    }

    ~HumanRestamper() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    void callback(const saphari_msgs::Human::ConstPtr& msg)
    {
      publisher_.publish(restamp(*msg));
    }

    saphari_msgs::Human restamp(const saphari_msgs::Human& in_msg)
    {
      saphari_msgs::Human out_msg(in_msg);
      out_msg.header.stamp = ros::Time::now(); 
      return out_msg;
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_stamper");

  ros::NodeHandle nh("~");

  HumanRestamper human_stamper(nh);

  ros::spin();

  return 0;
}
