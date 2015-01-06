/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <urdf/model.h>

#include <OgreSceneNode.h>
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/robot/robot.h"
#include "rviz/display_context.h"
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
  , time_since_last_transform_( 0.0f )
{ 
  visual_enabled_property_ = new rviz::Property( "Visual Enabled", true,
                                                 "Whether to display the visual representation of the robot.",
                                                 this, SLOT( updateVisualVisible() ));

  collision_enabled_property_ = new rviz::Property( "Collision Enabled", false,
                                                    "Whether to display the collision representation of the robot.",
                                                    this, SLOT( updateCollisionVisible() ));

  update_rate_property_ = new rviz::FloatProperty( "Update Interval", 0,
                                                   "Interval at which to update the links, in seconds. "
                                                   " 0 means to update every update cycle.",
                                                   this );
  update_rate_property_->setMin( 0 );

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1,
                                             "Amount of transparency to apply to the links.",
                                              this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0.0 );
  alpha_property_->setMax( 1.0 );

  robot_description_topic_property_ = new rviz::RosTopicProperty( "Robot description topic", "dynamic_robot_description",
                                                                  QString::fromStdString( ros::message_traits::datatype<std_msgs::String>() ),
                                                                  "std_msgs::String topic to subscribe to.",
                                                                  this, SLOT( updateTopic() ));

  tf_prefix_property_ = new rviz::StringProperty( "TF Prefix", "",
                                                  "Robot Model normally assumes the link name is the same as the tf frame name. "
                                                  " This option allows you to set a prefix. Mainly useful for multi-robot situations.",
                                                  this, SLOT( updateTfPrefix() ));
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
  robot_->setVisible( true );
}

void DynamicRobotModel::onDisable()
{
  sub_.shutdown();
  robot_->setVisible( false );
}

void DynamicRobotModel::update( float wall_dt, float ros_dt )
{
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

  if( update )
  {
    robot_->update( rviz::TFLinkUpdater( context_->getFrameManager(),
                                         boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
                                         tf_prefix_property_->getStdString() ));
    context_->queueRender();
    time_since_last_transform_ = 0.0f;
  }
}

void DynamicRobotModel::updateTopic()
{
  subscribe();
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
  context_->queueRender();
}

void DynamicRobotModel::updateAlpha()
{
  robot_->setAlpha( alpha_property_->getFloat() );
  context_->queueRender();
}

void DynamicRobotModel::updateVisualVisible()
{
  robot_->setVisualVisible( visual_enabled_property_->getValue().toBool() );
  context_->queueRender();
}

void DynamicRobotModel::updateCollisionVisible()
{
  robot_->setCollisionVisible( collision_enabled_property_->getValue().toBool() );
  context_->queueRender();
}

void DynamicRobotModel::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
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

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( dynamic_robot_model_rviz_plugin::DynamicRobotModel, rviz::Display )