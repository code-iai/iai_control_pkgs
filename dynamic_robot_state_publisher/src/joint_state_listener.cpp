/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include "dynamic_robot_state_publisher/joint_state_listener.h"

JointStateListener::JointStateListener(const KDL::Tree& tree, const MimicMap& m)
  : state_publisher_(tree), mimic_(m)
{
  ros::NodeHandle nh("~");

  // set publish frequency
  double publish_freq;
  nh.param("publish_frequency", publish_freq, 50.0);

  // get the tf_prefix parameter from the closest namespace
  std::string tf_prefix_key;
  nh.searchParam("tf_prefix", tf_prefix_key);
  nh.param(tf_prefix_key, tf_prefix_, std::string(""));
  publish_interval_ = ros::Duration(1.0/std::max(publish_freq,1.0));

  // get whether to publish static-tfs
  nh.param("publish_static_tf", publish_static_tf_, false);

  // subscribe to joint state
  ROS_DEBUG("Subscribing to joint_states.");
  joint_state_sub_ = nh.subscribe("joint_states", 1, &JointStateListener::callbackJointState, this);
  // trigger to publish fixed joints
  timer_ = nh.createTimer(publish_interval_, &JointStateListener::callbackFixedJoint, this);

};

JointStateListener::~JointStateListener() 
{
};

void JointStateListener::callbackFixedJoint(const ros::TimerEvent& e)
{
  state_publisher_.publishFixedTransforms(tf_prefix_, publish_static_tf_);
}

void JointStateListener::callbackJointState(const JointStateConstPtr& state)
{
  if (state->name.size() != state->position.size()){
    ROS_ERROR("Robot state publisher received an invalid joint state vector");
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  ros::Time now = ros::Time::now();
  if(last_callback_time_ > now) {
    // force re-publish of joint transforms
    ROS_WARN("Moved backwards in time (probably because ROS clock was reset), re-publishing joint transforms!");
    last_publish_time_.clear();
  }
  last_callback_time_ = now;

  // determine least recently published joint
  ros::Time last_published = now;
  for (unsigned int i=0; i<state->name.size(); i++)
  {
    ros::Time t = last_publish_time_[state->name[i]];
    last_published = (t < last_published) ? t : last_published;
  }

  // note: if a joint was seen for the first time,
  // then last_published is zero.
  // check if we need to publish
  if (state->header.stamp >= last_published + publish_interval_){
    // get joint positions from state message
    std::map<std::string, double> joint_positions;
    for (unsigned int i=0; i<state->name.size(); i++)
      joint_positions.insert(make_pair(state->name[i], state->position[i]));

    for(MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++){
      if(joint_positions.find(i->second->joint_name) != joint_positions.end()){
        double pos = joint_positions[i->second->joint_name] * i->second->multiplier + i->second->offset;
        joint_positions.insert(make_pair(i->first, pos));
      }
    }

    state_publisher_.publishTransforms(joint_positions, state->header.stamp, tf_prefix_);

    // store publish time in joint map
    for (unsigned int i=0; i<state->name.size(); i++)
    last_publish_time_[state->name[i]] = state->header.stamp;
  }
}
