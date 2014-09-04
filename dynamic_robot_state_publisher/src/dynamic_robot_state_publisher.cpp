#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include "dynamic_robot_state_publisher/joint_state_listener.h"
#include "dynamic_robot_state_publisher/dynamic_robot_state_publisher.h"
#include "dynamic_robot_state_publisher/AlterUrdf.h"
#include "tinyxml.h"

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

  // create the xml document
  xml_doc_.reset(new TiXmlDocument ("robot_description.urdf"));
  xml_doc_->Parse((char*)urdf_description.c_str());

  // create a joint_state_listener
  createNewJointStateListener();

  // create the service
  ros::ServiceServer service = nh.advertiseService("alter_urdf", &DynamicRobotStatePublisher::callbackAlterUrdf, this);
  
  ros::spin();

};

DynamicRobotStatePublisher::~DynamicRobotStatePublisher()
{};

bool DynamicRobotStatePublisher::callbackAlterUrdf(dynamic_robot_state_publisher::AlterUrdf::Request &req,
    dynamic_robot_state_publisher::AlterUrdf::Response &res)
{
  ROS_INFO("Action %i", req.action);

  ROS_INFO("Reset JointStateListener");
  joint_state_listener_.reset();

  // add the new links and joints to the xml
  if (req.action == dynamic_robot_state_publisher::AlterUrdf::Request::ADD) {
    ROS_INFO("Input xml: %s", req.xml.c_str());

    // parse the xml input
    TiXmlDocument xml_addition("additional_links_joints.urdf");
    xml_addition.Parse(req.xml.c_str());

    ROS_INFO("Parsed xml:");
    xml_addition.Print();

    // add the xml elements to the urdf
    TiXmlNode *previous_child = xml_addition.FirstChild();
    while (previous_child)
    {
      ROS_INFO("Adding %s %s", previous_child->Value(), previous_child->ToElement()->Attribute("name"));
      xml_doc_->RootElement()->LinkEndChild(previous_child->Clone());
      previous_child = xml_addition.IterateChildren(previous_child);
    } 
  }

  if (req.action == dynamic_robot_state_publisher::AlterUrdf::Request::REMOVE) {
    // remove elements with from the urdf which names are in req.names
    for(std::vector<std::string>::iterator name = req.names.begin(); name != req.names.end(); ++name)
    {
      ROS_INFO("Searching %s", (*name).c_str());
      TiXmlNode* child = find_child_with_attribute(xml_doc_->RootElement()->FirstChildElement(), "name", *name);
      if (child) {
        ROS_INFO("Removing %s", (*name).c_str());
        xml_doc_->RootElement()->RemoveChild(child);
      }
    }
  }

  ROS_INFO("New urdf:");
  xml_doc_->Print();

  // create an updated joitn state listener
  createNewJointStateListener();

  res.success = true;
  return true;
}

void DynamicRobotStatePublisher::createNewJointStateListener()
{
  ROS_INFO("Creating new joint state listener.");

  // create the urdf model from the xml document
  urdf::Model model;
  model.initXml(xml_doc_.get());

  ROS_INFO("debug3");
  // create the kdl tree from the model
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return;
  }

  ROS_INFO("debug4");
  MimicMap mimic;
  for (std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++)
  {
    if (i->second->mimic) {
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }

  ROS_INFO("debug6");
  joint_state_listener_.reset(new JointStateListener(tree, mimic));
}

TiXmlElement* DynamicRobotStatePublisher::find_child_with_attribute(TiXmlElement* first_child_element, const char *attribute, std::string value)
{
  TiXmlElement* sibling = first_child_element;
  std::string *attribute_value;
  while (sibling) 
  {
    ROS_INFO("debug0");
    //sibling->QueryStringAttribute(attribute, attribute_value); 
    ROS_INFO("debug1");
    if (strcmp(sibling->Attribute(attribute), value.c_str()) == 0)
      return sibling;
    ROS_INFO("debug2");
    sibling = sibling->NextSiblingElement();
  }

  return NULL;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_robot_state_publisher");

  DynamicRobotStatePublisher dynamic_publisher;

  return 0;
}