#ifndef DYNAMIC_ROBOT_MODEL_H
#define DYNAMIC_ROBOT_MODEL_H

//The following ifndef should fix usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"
// Reference: https://github.com/ros-planning/moveit_ros/issues/653
#ifndef Q_MOC_RUN 
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "rviz/robot/robot.h"
#include "rviz/display.h"
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class RosTopicProperty;
class FloatProperty;
class Property;
class StringProperty;
class Robot;
}

namespace dynamic_robot_model_rviz_plugin
{

class DynamicRobotModel: public rviz::Display
{
Q_OBJECT
public:
  DynamicRobotModel();
  virtual ~DynamicRobotModel();
  virtual void update( float wall_dt, float ros_dt );

private Q_SLOTS:
  virtual void updateTopic();
  virtual void updateVisualVisible();
  virtual void updateCollisionVisible();
  virtual void updateTfPrefix();
  virtual void updateAlpha();

protected:
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void processMessage( const std_msgs::String::ConstPtr& msg );
  virtual void updateRobot();
  virtual void subscribe();

  rviz::Robot* robot_;

  float time_since_last_transform_;

  std::string robot_description_;

  ros::Subscriber sub_;

  rviz::RosTopicProperty* robot_description_topic_property_;
  rviz::Property* visual_enabled_property_;
  rviz::Property* collision_enabled_property_;
  rviz::FloatProperty* update_rate_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::StringProperty* tf_prefix_property_;

};
}

#endif
