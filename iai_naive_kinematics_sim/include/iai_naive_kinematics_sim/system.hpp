#ifndef IAI_NAIVE_KINEMATICS_SIM_SYSTEM_HPP
#define IAI_NAIVE_KINEMATICS_SIM_SYSTEM_HPP

#include <iai_naive_kinematics_sim/simulator.hpp>
#include <iai_naive_kinematics_sim/watchdog.hpp>

namespace iai_naive_kinematics_sim
{
  class System
  {
    public:
      System() {}
      ~System() {}

      void init(const urdf::Model& model, const std::vector<std::string>& controlled_joints,
          double watchdog_period)
      {
        for(size_t i=0; i<controlled_joints.size(); ++i)
          // TODO: throw exception
          assert(modelHasMovableJoint(model, controlled_joints[i]));
        
        sim_.init(model);

        dogs_.clear();
        for(size_t i=0; i<controlled_joints.size(); ++i)
          dogs_[controlled_joints[i]] = Watchdog(ros::Duration(watchdog_period));
      }

      const sensor_msgs::JointState& getJointState() const
      {
        return sim_.getJointState();
      }

      void setSubJointState(const sensor_msgs::JointState& state)
      {
        sim_.setSubJointState(state);
      }

      void update(const ros::Time& now, double dt)
      {
        for(std::map<std::string, Watchdog>::iterator it=dogs_.begin(); it!=dogs_.end(); ++it)
        {
          it->second.update(now);
          sim_.setNextJointVelocity(it->first, it->second.getCommand());
        }
        sim_.update(now, dt);
      }

      void setVelocityCommand(const std::string& name, double velocity, const ros::Time& now)
      {
        std::map<std::string, Watchdog>::iterator it = dogs_.find(name);

        // TODO: throw exception
        assert(it!=dogs_.end());

        it->second.setNewCommand(now, velocity);
      }

      const SimulatorVelocityResolved& getSim() const
      {
        return sim_;
      }

      const std::map<std::string, Watchdog>& getWatchdogs() const
      {
        return dogs_;
      }

    private:
      SimulatorVelocityResolved sim_;
      std::map<std::string, Watchdog> dogs_;
  };
}

#endif
