#ifndef IAI_NAIVE_KINEMATICS_SIM_SIMULATOR_HPP
#define IAI_NAIVE_KINEMATICS_SIM_SIMULATOR_HPP

#include <vector>
#include <map>
#include <string>
#include <sensor_msgs/JointState.h>

namespace iai_naive_kinematics_sim
{
  class SimulatorVelocityResolved
  {
    public:
      SimulatorVelocityResolved() {}

      ~SimulatorVelocityResolved() {}

      void init(const std::vector<std::string>& joint_names)
      {
        resize(joint_names.size());
        state_.name = joint_names;
        zero();
        recalculate_index_map(joint_names);
      }

      void init(const sensor_msgs::JointState& state)
      {
        state_ = state;
        recalculate_index_map(state.name);
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

        state_.header.stamp = now;
        state_.header.seq++;
      }

      const sensor_msgs::JointState& getJointState() const
      {
        return state_;
      }

      void setNextJointVelocity(const std::string& joint_name, double velocity)
      {
        std::map<std::string, size_t>::iterator it = index_map_.find(joint_name);
        
        // TODO: throw exception
        assert(it!=index_map_.end());

        state_.velocity[it->second] = velocity;
      }

    private:
      // internal state of the simulator
      sensor_msgs::JointState state_;

      // a map from joint-state names to their index in the joint-state message
      std::map<std::string, size_t> index_map_;

      void resize(size_t new_size)
      {
        state_.name.resize(new_size);
        state_.position.resize(new_size);
        state_.velocity.resize(new_size);
        state_.effort.resize(new_size);
      }

      void zero()
      {
        for(size_t i=0; i<state_.position.size(); ++i)
          state_.position[i] = 0.0;
        for(size_t i=0; i<state_.velocity.size(); ++i)
          state_.velocity[i] = 0.0;
        for(size_t i=0; i<state_.effort.size(); ++i)
          state_.effort[i] = 0.0;
      }

      void recalculate_index_map(const std::vector<std::string>& joint_names)
      {
        index_map_.clear();
        for(size_t i=0; i<joint_names.size(); ++i)
          index_map_[joint_names[i]] = i;
      }
  };
}

#endif
