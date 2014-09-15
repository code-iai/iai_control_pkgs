#ifndef DYNAMIC_ROBOT_STATE_PUBLISHER_H
#define DYNAMIC_ROBOT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "dynamic_robot_state_publisher/joint_state_listener.h"
#include "dynamic_robot_state_publisher/AlterUrdf.h"
#include "tinyxml.h"

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
  bool createNewJointStateListener(boost::shared_ptr<TiXmlDocument> urdf_xml);
  TiXmlElement* find_child_with_attribute(TiXmlElement* first_child_element, const char *attribute, std::string value); 

	//ros::ServiceServer alter_urdf_sub_;
  boost::shared_ptr<JointStateListener> joint_state_listener_;
  boost::shared_ptr<TiXmlDocument> xml_doc_;

};

#endif