#ifndef DYNAMIC_ROBOT_STATE_PUBLISHER_H
#define DYNAMIC_ROBOT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "dynamic_robot_state_publisher/joint_state_listener.h"
#include "dynamic_robot_state_publisher/AlterUrdf.h"
//#include <tinyxml/tinyxml.h>

typedef std::map<std::string, boost::shared_ptr<urdf::JointMimic> > MimicMap;
typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;

class DynamicRobotStatePublisher
{
public:
  /// Construtor
	DynamicRobotStatePublisher();

  /// Destructor
	~DynamicRobotStatePublisher();

private:
  bool callbackAlterUrdf(dynamic_robot_state_publisher::AlterUrdf::Request &req,
    dynamic_robot_state_publisher::AlterUrdf::Response &res);

	//ros::ServiceServer alter_urdf_sub_;
  //dynamic_robot_state_publisher::JointStateListener joint_state_listener_;

};

#endif