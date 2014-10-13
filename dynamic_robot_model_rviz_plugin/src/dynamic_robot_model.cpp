#include <ros/ros.h>
#include <rviz/properties/ros_topic_property.h>
#include "std_msgs/String.h"
#include "rviz/robot/robot.h"
#include "rviz/display.h"
#include "rviz/display_context.h"
#include "tinyxml.h"
#include <urdf/model.h>
#include "rviz/robot/tf_link_updater.h"
#include "dynamic_robot_model.h"


namespace dynamic_robot_model_rviz_plugin
{

void linkUpdaterStatusFunction( rviz::StatusProperty::Level level,
                                const std::string& link_name,
                                const std::string& text,
                                rviz::Display* display )
{
  display->setStatus( level, QString::fromStdString( link_name ), QString::fromStdString( text ));
}

DynamicRobotModel::DynamicRobotModel()
  : rviz::Display()
{
  robot_description_topic_property_ = new rviz::RosTopicProperty( "Robot description topic", "dynamic_robot_description",
                                                                  QString::fromStdString( ros::message_traits::datatype<std_msgs::String>() ),
                                                                  "std_msgs::String topic to subscribe to.",
                                                                  this, SLOT( updateTopic() ));
}

DynamicRobotModel::~DynamicRobotModel()
{
  sub_.shutdown();
}

void DynamicRobotModel::onInitialize()
{
  robot_ = new rviz::Robot( scene_node_, context_, "Robot: " + getName().toStdString(), this );
  context_->queueRender();
}

void DynamicRobotModel::onEnable()
{
  subscribe();
}

void DynamicRobotModel::updateTopic()
{
  subscribe();
}

void DynamicRobotModel::subscribe()
{
  if( !isEnabled() )
  {
    return;
  }

  std::string topic_name = robot_description_topic_property_->getTopicStd();
  if( !topic_name.empty() ) 
  {
    sub_.shutdown();

    try
    {
      sub_ = update_nh_.subscribe( topic_name, 100, &DynamicRobotModel::processMessage, this );
      setStatus( rviz::StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what() );
    }
  }
}

void DynamicRobotModel::processMessage( const std_msgs::String::ConstPtr& msg )
{
  robot_description_ = msg->data;
  updateRobot();
}

void DynamicRobotModel::updateRobot()
{
  TiXmlDocument doc;
  doc.Parse( robot_description_.c_str() );

  if( !doc.RootElement() )
  {
    robot_->clear();
    setStatus( rviz::StatusProperty::Error, "URDF", "URDF failed XML parse" );
    return;
  }

  urdf::Model descr;
  if( !descr.initXml( doc.RootElement() ))
  {
    robot_->clear();
    setStatus( rviz::StatusProperty::Error, "URDF", "URDF failed Model parse" );
    return;
  }

  setStatus( rviz::StatusProperty::Ok, "URDF", "URDF parsed OK" );
  robot_->load( descr );
  robot_->update( rviz::TFLinkUpdater( context_->getFrameManager(),
                                       boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
                                       "" ));

  context_->queueRender();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dynamic_robot_model_rviz_plugin::DynamicRobotModel, rviz::Display)


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "test_foo");

//   std::cout << "Start";
//   dynamic_robot_model_rviz_plugin::DynamicRobotModel dyn;
//   std::cout << "Ende";

//   return 0;
// }