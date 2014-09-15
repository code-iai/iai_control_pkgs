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
  createNewJointStateListener(xml_doc_);

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

  boost::shared_ptr<TiXmlDocument> xml_new_doc(new TiXmlDocument(*xml_doc_));

  // add the new links and joints to the xml
  if (req.action == dynamic_robot_state_publisher::AlterUrdf::Request::ADD) {
    ROS_INFO("Input xml: %s", req.xml.c_str());

    // parse the xml input
    TiXmlDocument xml_addition("additional_links_joints.urdf");
    xml_addition.Parse(req.xml.c_str());

    // check if an error occured
    if (xml_addition.Error()) {
      ROS_ERROR("Error: %s", xml_addition.ErrorDesc());
      res.success = false;
      return true;
    }

    ROS_INFO("Parsed xml:");
    xml_addition.Print();

    // add the xml elements to the urdf
    TiXmlNode *previous_child = xml_addition.FirstChild();
    while (previous_child)
    {
      ROS_INFO("Adding %s %s", previous_child->Value(), previous_child->ToElement()->Attribute("name"));
      xml_new_doc->RootElement()->LinkEndChild(previous_child->Clone());
      previous_child = xml_addition.IterateChildren(previous_child);
    } 
  }

  // remove links and joints from the urdf
  if (req.action == dynamic_robot_state_publisher::AlterUrdf::Request::REMOVE) {
    TiXmlNode* child;

    // remove elements form the urdf which names are in req.names
    for(std::vector<std::string>::iterator name = req.names.begin(); name != req.names.end(); ++name)
    {
      ROS_INFO("Searching %s", (*name).c_str());
      child = find_child_with_attribute(xml_new_doc->RootElement()->FirstChildElement(), "name", *name);
      if (child) {
        ROS_INFO("Removing %s", (*name).c_str());
        xml_new_doc->RootElement()->RemoveChild(child);
      } 
      else {
        ROS_INFO("Couldn't find %s", (*name).c_str());
      }
    }
  }

  ROS_INFO("New urdf:");
  xml_new_doc->Print();

  // create an updated joint state listener
  res.success = createNewJointStateListener(xml_new_doc);

  if (res.success) {
    xml_doc_ = xml_new_doc;
  }

  return true;
}

bool DynamicRobotStatePublisher::createNewJointStateListener(boost::shared_ptr<TiXmlDocument> urdf_xml)
{
  ROS_INFO("Creating new joint state listener.");

  // create the urdf model from the xml document
  urdf::Model model;
  if (!model.initXml(urdf_xml.get())) {
    ROS_ERROR("Couldn't create urdf model from xml.");
    return false;
  }

  // create the kdl tree from the model
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return false;
  }

  MimicMap mimic;
  for (std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++)
  {
    if (i->second->mimic) {
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }

  joint_state_listener_.reset(new JointStateListener(tree, mimic));

  return true;
}

TiXmlElement* DynamicRobotStatePublisher::find_child_with_attribute(TiXmlElement* first_child_element, const char *attribute, std::string value)
{
  TiXmlElement* sibling = first_child_element;
  std::string *attribute_value;
  while (sibling) 
  {
    if (strcmp(sibling->Attribute(attribute), value.c_str()) == 0)
      return sibling;

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