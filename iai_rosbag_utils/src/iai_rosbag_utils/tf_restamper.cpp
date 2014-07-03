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
    geometry_msgs::TransformStamped shoulder_kinect_link;
    geometry_msgs::TransformStamped shoulder_kinect_rgb_frame;

    void callback(const tf2_msgs::TFMessage::ConstPtr& msg)
    {
      publisher_.publish(restamp(*msg));
    }

    tf2_msgs::TFMessage restamp(const tf2_msgs::TFMessage& in_msg)
    {
      tf2_msgs::TFMessage out_msg(in_msg);
      int updated_link = 0;
      int updated_rgb_frame = 0;

      for (std::vector<geometry_msgs::TransformStamped>::iterator it = out_msg.transforms.begin(); it != out_msg.transforms.end(); it++)
      {
        it->header.stamp = ros::Time::now();

        /*
        if(it->child_frame_id == "/shoulder_kinect_link")
        {
          shoulder_kinect_link = *it;
          updated_link = 1;
        }

        if(it->child_frame_id == "/shoulder_kinect_rgb_frame")
        {
          shoulder_kinect_rgb_frame = *it;
          updated_rgb_frame = 1;
        }
        */
          
      }

      /*
      if(!updated_link) 
      {
        shoulder_kinect_link.header.stamp = ros::Time::now();
        out_msg.transforms.push_back(shoulder_kinect_link);
      }

      if(!updated_rgb_frame) 
      {
        shoulder_kinect_rgb_frame.header.stamp = ros::Time::now();
        out_msg.transforms.push_back(shoulder_kinect_rgb_frame);
      }
      */

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
