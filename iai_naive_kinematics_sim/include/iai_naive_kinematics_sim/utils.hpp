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
