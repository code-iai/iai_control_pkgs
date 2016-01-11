/*
 * Copyright (c) 2015, Georg Bartels, <georg.bartels@cs.uni-bremen.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Institute of Artificial Intelligence, 
 *     University of Bremen nor the names of its contributors may be used 
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IAI_NAIVE_KINEMATICS_SIM_UTILS_HPP
#define IAI_NAIVE_KINEMATICS_SIM_UTILS_HPP

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

namespace iai_naive_kinematics_sim
{
  inline void setJointName(sensor_msgs::JointState& state, size_t index, const std::string& name)
  {
    // TODO: throw exception
    assert(index < state.name.size());
    state.name[index] = name;
  }

  inline void setJointPosition(sensor_msgs::JointState& state, size_t index, double position)
  {
    // TODO: throw exception
    assert(index < state.position.size());
    state.position[index] = position;
  }

  inline void setJointVelocity(sensor_msgs::JointState& state, size_t index, double velocity)
  {
    // TODO: throw exception
    assert(index < state.velocity.size());
    state.velocity[index] = velocity;
  }

  inline void setJointEffort(sensor_msgs::JointState& state, size_t index, double effort)
  {
    // TODO: throw exception
    assert(index < state.effort.size());
    state.effort[index] = effort;
  }

  inline void setJointState(sensor_msgs::JointState& state, size_t index, const std::string& name,
      double position, double velocity, double effort)
  {
    setJointName(state, index, name);
    setJointPosition(state, index, position);
    setJointVelocity(state, index, velocity);
    setJointEffort(state, index, effort);
  }

  inline void pushBackJointState(sensor_msgs::JointState& state, const std::string& name,
      double position, double velocity, double effort)
  {
    state.name.push_back(name);
    state.position.push_back(position);
    state.velocity.push_back(velocity);
    state.effort.push_back(effort);
  }

  inline void clearJointState(sensor_msgs::JointState& state)
  {
    state.name.clear();
    state.position.clear();
    state.velocity.clear();
    state.effort.clear();
  }

  inline bool isMovingJoint(int type)
  {
    return (type == urdf::Joint::REVOLUTE ||
            type == urdf::Joint::CONTINUOUS ||
            type == urdf::Joint::PRISMATIC);
  }

  inline bool modelHasMovableJoint(const urdf::Model& model, const std::string& name)
  {
    boost::shared_ptr<const urdf::Joint> joint = model.getJoint(name);
    return joint.get() && isMovingJoint(joint->type);
  }

  inline std::map<std::string, size_t> makeJointIndexMap(const std::vector<std::string>& joint_names)
  {
    std::map<std::string, size_t> map;
    for(size_t i=0; i<joint_names.size(); ++i)
      map[joint_names[i]] = i;
    return map;
  }

  inline sensor_msgs::JointState bootstrapJointState(const urdf::Model& model)
  {
    sensor_msgs::JointState state;

    for(std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator it=model.joints_.begin(); it!=model.joints_.end(); ++it)
      if(isMovingJoint(it->second->type))
        pushBackJointState(state, it->second->name, 0.0, 0.0, 0.0);

    return state;
  }

}

#endif
