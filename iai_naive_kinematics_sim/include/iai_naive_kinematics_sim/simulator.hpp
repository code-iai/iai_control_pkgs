#ifndef IAI_NAIVE_KINEMATICS_SIM_SIMULATOR_HPP
#define IAI_NAIVE_KINEMATICS_SIM_SIMULATOR_HPP

#include <iai_naive_kinematics_sim/utils.hpp>

namespace iai_naive_kinematics_sim
{
 
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
        assert(state_.name.size() == state_.position.size());
        assert(state_.position.size() == state_.velocity.size());

        for(size_t i=0; i<state_.position.size(); ++i)
        {
          state_.position[i] += state_.velocity[i] * dt;
          enforceJointLimits(state_.name[i]);
        } 

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

      boost::shared_ptr<urdf::Joint> getJoint(const std::string& name) const
      {
        std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator it =
          model_.joints_.find(name);

        // TODO: throw exception
        assert(it != model_.joints_.end());

        return it->second;
      }

      void enforceJointLimits(const std::string& name)
      {
        boost::shared_ptr<urdf::Joint> joint = getJoint(name);

        if ((joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) &&
            joint->limits.get())
        {
          // joint should be limited, and has limits specified
          
          size_t index = getJointIndex(name);
          if(state_.position[index] < joint->limits->lower ||
             state_.position[index] > joint->limits->upper)
          {
            state_.position[index] = 
              std::max(joint->limits->lower, std::min(state_.position[index], joint->limits->upper));
            state_.velocity[index] = 0.0;
          }
        }
      }
  };
}

#endif
