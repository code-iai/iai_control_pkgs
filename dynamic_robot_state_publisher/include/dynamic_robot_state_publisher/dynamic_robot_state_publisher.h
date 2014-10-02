#ifndef DYNAMIC_ROBOT_STATE_PUBLISHER_H
#define DYNAMIC_ROBOT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "dynamic_robot_state_publisher/joint_state_listener.h"
#include "std_msgs/String.h"

class DynamicRobotStatePublisher
{
public:
  /// Construtor
	DynamicRobotStatePublisher();

  /// Destructor
	~DynamicRobotStatePublisher();

private:
  void callbackDynamicRobotDescription(const std_msgs::String::ConstPtr& robot_description);
  ros::Subscriber sub_;
  boost::shared_ptr<JointStateListener> joint_state_listener_;

};

#endif