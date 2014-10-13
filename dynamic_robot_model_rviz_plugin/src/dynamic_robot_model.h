#ifndef DYNAMIC_ROBOT_MODEL_H
#define DYNAMIC_ROBOT_MODEL_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "rviz/robot/robot.h"
#include "rviz/display.h"


namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class RosTopicProperty;
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
  //virtual void update( float wall_dt, float ros_dt );

protected:
  virtual void onInitialize();
  virtual void onEnable();

  rviz::Robot* robot_;

private Q_SLOTS:
  virtual void updateTopic();

private:
  virtual void processMessage( const std_msgs::String::ConstPtr& msg );
  virtual void updateRobot();
  virtual void subscribe();

  rviz::RosTopicProperty* robot_description_topic_property_;
  std::string robot_description_;
  ros::Subscriber sub_;

};
}

#endif