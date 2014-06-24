#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

class TFRestamper
{
  public:
    TFRestamper(const ros::NodeHandle& nh): nh_(nh)
    {
      publisher_ = nh_.advertise<tf2_msgs::TFMessage>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &TFRestamper::callback, this);
    }

    ~TFRestamper() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    void callback(const tf2_msgs::TFMessage::ConstPtr& msg)
    {
      publisher_.publish(restamp(*msg));
    }

    tf2_msgs::TFMessage restamp(const tf2_msgs::TFMessage& in_msg)
    {
      tf2_msgs::TFMessage out_msg(in_msg);

      for (std::vector<geometry_msgs::TransformStamped>::iterator it = out_msg.transforms.begin(); it != out_msg.transforms.end(); it++)
      {
        it->header.stamp = ros::Time::now();
      } 

      return out_msg;
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_restamper");

  ros::NodeHandle nh("~");

  TFRestamper tf_restamper(nh);

  ros::spin();

  return 0;
}
