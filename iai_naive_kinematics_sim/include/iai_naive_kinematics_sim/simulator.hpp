#ifndef IAI_NAIVE_KINEMATICS_SIM_SIMULATOR_HPP
#define IAI_NAIVE_KINEMATICS_SIM_SIMULATOR_HPP

#include <vector>
#include <map>
#include <string>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

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

  class SimulatorVelocityResolved
  {
    public:
      SimulatorVelocityResolved() {}

      ~SimulatorVelocityResolved() {}

      void init(const urdf::Model& model)
      {
        model_ = model;
        state_ = bootstrapJointState(model);
        index_map_ = makeJointIndexMap(state_.name);
      }

      size_t size() const
      {
        return index_map_.size();
      }

      void update(const ros::Time& now, double dt)
      {
        // TODO: throw exceptions
        assert(dt > 0);
        assert(state_.position.size() == state_.velocity.size());

        for(size_t i=0; i<state_.position.size(); ++i)
          state_.position[i] += state_.velocity[i] * dt;

        // TODO: enforce position limits

        state_.header.stamp = now;
        state_.header.seq++;
      }

      const sensor_msgs::JointState& getJointState() const
      {
        return state_;
      }

      void setSubJointState(const sensor_msgs::JointState& state)
      {
        // TODO: throw exceptions
        assert(state.name.size() == state.position.size());
        assert(state.name.size() == state.velocity.size());

        for(size_t i=0; i<state.name.size(); ++i)
          setJointState(state_, getJointIndex(state.name[i]), state.name[i], 
              state.position[i], state.velocity[i], state.effort[i]);
      }

      void setNextJointVelocity(const std::string& joint_name, double velocity)
      {
        setJointVelocity(state_, getJointIndex(joint_name), velocity);
      }

    private:
      // internal state of the simulator
      sensor_msgs::JointState state_;

      // a map from joint-state names to their index in the joint-state message
      std::map<std::string, size_t> index_map_;

      // urdf model to lookup information about the joints
      urdf::Model model_;

      size_t getJointIndex(const std::string& name) const
      {
        std::map<std::string, size_t>::const_iterator it = index_map_.find(name);
        
        // TODO: throw exception
        assert(it!=index_map_.end());

        return it->second;
      }
  };
}

#endif
