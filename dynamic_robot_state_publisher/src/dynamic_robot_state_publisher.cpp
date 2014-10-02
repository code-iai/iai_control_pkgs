#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include "dynamic_robot_state_publisher/joint_state_listener.h"
#include "dynamic_robot_state_publisher/dynamic_robot_state_publisher.h"
#include "std_msgs/String.h"


DynamicRobotStatePublisher::DynamicRobotStatePublisher() 
{
  ros::NodeHandle nh("~");

  // create the subscriber
  ros::Subscriber sub_ = nh.subscribe("in_topic", 100, &DynamicRobotStatePublisher::callbackDynamicRobotDescription, this);
  
  ros::spin();
};

DynamicRobotStatePublisher::~DynamicRobotStatePublisher()
{};

void DynamicRobotStatePublisher::callbackDynamicRobotDescription(const std_msgs::String::ConstPtr& robot_description)
{
  ROS_INFO("Creating new joint state listener.");

  // create the urdf model from the message
  urdf::Model model;
  if (!model.initString(robot_description->data.c_str())) {
    ROS_ERROR("Couldn't create urdf model from xml.");
    return;
  }

  // create the kdl tree from the model
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return;
  }

  MimicMap mimic;
  for (std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++)
  {
    if (i->second->mimic) {
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }

  joint_state_listener_.reset(new JointStateListener(tree, mimic));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_robot_state_publisher");

  DynamicRobotStatePublisher dynamic_publisher;

  return 0;
}