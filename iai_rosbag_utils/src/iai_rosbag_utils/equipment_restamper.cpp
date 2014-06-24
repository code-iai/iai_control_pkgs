#include <ros/ros.h>
#include <saphari_msgs/PerceivedEquipment.h>
#include <saphari_msgs/Equipment.h>

class EquipmentRestamper
{
  public:
    EquipmentRestamper(const ros::NodeHandle& nh): nh_(nh)
    {
      publisher_ = nh_.advertise<saphari_msgs::PerceivedEquipment>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &EquipmentRestamper::callback, this);
    }

    ~EquipmentRestamper() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    void callback(const saphari_msgs::PerceivedEquipment::ConstPtr& msg)
    {
      publisher_.publish(restamp(*msg));
    }

    saphari_msgs::PerceivedEquipment restamp(const saphari_msgs::PerceivedEquipment& in_msg)
    {
      saphari_msgs::PerceivedEquipment out_msg(in_msg);

      for (std::vector<saphari_msgs::Equipment>::iterator it = out_msg.perceived.begin(); it != out_msg.perceived.end(); it++)
      {
        it->pose.header.stamp = ros::Time::now();
      }

      return out_msg;
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "equipment_restamper");

  ros::NodeHandle nh("~");

  EquipmentRestamper equipment_restamper(nh);

  ros::spin();

  return 0;
}
