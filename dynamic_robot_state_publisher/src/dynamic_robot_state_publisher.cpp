#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include "dynamic_robot_state_publisher/joint_state_listener.h"
#include "dynamic_robot_state_publisher/dynamic_robot_state_publisher.h"
#include "dynamic_robot_state_publisher/AlterUrdf.h"
//#include <tinyxml/tinyxml.h>

DynamicRobotStatePublisher::DynamicRobotStatePublisher() 
{
  ros::NodeHandle nh("~");

  // get the inital robot description
  std::string urdf_description;
  if(nh.getParam("/robot_description", urdf_description)) {
    ROS_INFO("Found robot description on parameter server.");
  }
  else {
    ROS_INFO("No robot description found. Using default.");
    urdf_description = "<robot name=\"bob\"><link name=\"default\"></link></robot>";
  }

  // // create the xml document
  // TIXmlDocument xml_doc("robot_description.urdf");
  // xml_doc.parse(urdf_description);

  // // create the urdf model from the xml document
  // urdf::Model model;
  // model.initXml(xml_doc);

  // // create the kdl tree from the model
  // KDL::Tree tree;
  // if (!kdl_parser::treeFromUrdfModel(model, tree))
  // {
  //   ROS_ERROR("Failed to extract kdl tree from xml robot description");
  //   return -1;
  // }

  // MimicMap mimic;

  // for(std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++){
  //   if(i->second->mimic){
  //     mimic.insert(make_pair(i->first, i->second->mimic));
  //   }
  // }

  // dynamic_robot_state_publisher::JointStateListener joint_state_listener(tree, mimic)

  ROS_INFO("Create");
  ros::ServiceServer service = nh.advertiseService("alter_urdf", &DynamicRobotStatePublisher::callbackAlterUrdf, this);
  ROS_INFO("Create");
  
  ros::spin();

};

DynamicRobotStatePublisher::~DynamicRobotStatePublisher()
{};

bool DynamicRobotStatePublisher::callbackAlterUrdf(dynamic_robot_state_publisher::AlterUrdf::Request &req,
    dynamic_robot_state_publisher::AlterUrdf::Response &res)
{
  ROS_INFO("Here");
  //res.succes = true;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_robot_state_publisher");

  DynamicRobotStatePublisher dynamic_publisher;

  return 0;
}