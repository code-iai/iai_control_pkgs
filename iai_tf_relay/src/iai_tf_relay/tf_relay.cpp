#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <vector>

class TfRelay
{
  public:
    TfRelay(const ros::NodeHandle& nh) : 
        nh_(nh)
    {
      subscriber_ = nh_.subscribe("in_topic", 1, &TfRelay::callback, this);
    }
  
    ~TfRelay() {}

  private:
    // ROS communication
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    tf::TransformBroadcaster broadcaster_;

    void callback(const tf::tfMessage::ConstPtr& msg)
    {
      broadcaster_.sendTransform(msg->transforms);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_relay");

  ros::NodeHandle nh("~");

  TfRelay my_relay(nh);  

  ros::spin();

  return 0;
}
