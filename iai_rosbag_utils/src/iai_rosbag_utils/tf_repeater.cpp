#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

class TFRepeater
{
  public:
    TFRepeater(const ros::NodeHandle& nh): nh_(nh)
    {
      // Check if parameters are set else return
      if(!nh_.getParam("parent_frame", parent_frame_) || !nh_.getParam("child_frame", child_frame_)) 
        return;

      publisher_ = nh_.advertise<tf2_msgs::TFMessage>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &TFRepeater::callback, this);
      has_transform_ = 0;
      time_of_last_transform_ = ros::Time(0.1);
      ros::Duration time_between_transforms_ = ros::Duration(0.1);
    }

    ~TFRepeater() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::string parent_frame_;
    std::string child_frame_;
    geometry_msgs::TransformStamped last_transform_;
    int has_transform_;
    ros::Time time_of_last_transform_;
    ros::Duration time_between_transforms_;

    void callback(const tf2_msgs::TFMessage::ConstPtr& msg)
    { 
      //TODO make it work
      
      /*
      if((ros::Time::now() - time_of_last_transform_) < time_between_transforms_)
      {
        return;
      }

      time_of_last_transform_ = ros::Time::now();
      */
      

      int updated_transform = 0;
      tf2_msgs::TFMessage tf_msg = *msg;

      for (std::vector<geometry_msgs::TransformStamped>::iterator it = tf_msg.transforms.begin(); it != tf_msg.transforms.end(); it++)
      {
        if(it->child_frame_id ==  child_frame_ && it->header.frame_id ==  parent_frame_)
        {
          has_transform_ = 1;
          updated_transform = 1;
          last_transform_ = *it;
        }
      }

      if(!updated_transform)
      {
        tf2_msgs::TFMessage transform_msg;
        last_transform_.header.stamp = ros::Time::now();
        transform_msg.transforms.push_back(last_transform_);
        publisher_.publish(transform_msg);
      }
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_repeater");

  ros::NodeHandle nh("~");

  TFRepeater tf_repeater(nh);

  ros::spin();

  return 0;
}
