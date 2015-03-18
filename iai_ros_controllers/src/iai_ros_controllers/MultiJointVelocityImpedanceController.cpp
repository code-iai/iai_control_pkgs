#include <iai_ros_controllers/MultiJointVelocityImpedanceController.h>
#include <pluginlib/class_list_macros.h>

namespace iai_ros_controllers
{
  typedef MultiJointVelocityImpedanceController MJVIC;
  typedef MultiJointVelocityImpedanceCommand MJVICommand;
  MJVIC::MultiJointVelocityImpedanceController() {} 
  MJVIC::~MultiJointVelocityImpedanceController() {}

  bool MJVIC::init(hardware_interface::EffortImpedanceJointInterface* hw, ros::NodeHandle &nh)
  {
    if(!nh.getParam("joints", joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam 'joints' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    if(joint_names_.size() == 0)
    {
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    if(!nh.getParam("default_stiffness", default_stiffness_))
    {
      ROS_ERROR_STREAM("Failed to getParam 'default_stiffness_' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    if(!nh.getParam("default_damping", default_damping_))
    {
      ROS_ERROR_STREAM("Failed to getParam 'default_damping_' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    double state_publish_rate;
    if(!nh.getParam("state_publish_rate", state_publish_rate))
    {
      ROS_ERROR_STREAM("Failed to getParam 'state_publish_rate' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    if(state_publish_rate <= 0.0)
    {
      ROS_ERROR_STREAM("State publish rate is not greater than 0.");
      return false;
    }

    state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

    double watchdog_period;
    if(!nh.getParam("watchdog_period", watchdog_period))
    {
      ROS_ERROR_STREAM("Failed to getParam 'watchdog_period' (namespace: " << nh.getNamespace() << ").");
      return false;
    }

    if(watchdog_period <= 0.0)
    {
      ROS_ERROR_STREAM("Watchdog period is not greater than 0.");
      return false;
    }

    watchdog_.setPeriod(ros::Duration(watchdog_period));

    for(unsigned int i=0; i<joint_names_.size(); i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }

    tmp_cmd_ = MJVICommand(joint_names_.size(), 0.0, 
      default_stiffness_, default_damping_);
    cmd_buffer_.set(tmp_cmd_);

    time_buffer_.set(ros::Time(0.0));

    cmd_sub_ = nh.subscribe<iai_control_msgs::MultiJointVelocityImpedanceCommand>(
      "command", 1, &MJVIC::callback, this);

    initStatePublisher(nh);

    return true;
  }

  void MJVIC::starting(const ros::Time& time)
  {
    last_state_publish_time_ = time;
    time_buffer_.set(time);
  }

  void MJVIC::update(const ros::Time& time, const ros::Duration& period)
  {
    // read and write buffers
    time_buffer_.set(time);
    cmd_buffer_.get(tmp_cmd_);

    if(watchdog_.isActive(time))
    {
      ROS_WARN_STREAM("Watchdog active.");
      for(unsigned int i=0; i<joint_names_.size(); i++)
      {
        tmp_cmd_.velocities_[i] = 0.0; 
        tmp_cmd_.stiffnesses_[i] = default_stiffness_; 
        tmp_cmd_.dampings_[i] = default_damping_; 
      }
    }
    
    // send commands to hardware
    for(unsigned int i=0; i<joint_names_.size(); i++)
    { 
      joints_[i].setCommand(tmp_cmd_.velocities_[i]); 
      joints_[i].setStiffness(tmp_cmd_.stiffnesses_[i]); 
      joints_[i].setDamping(tmp_cmd_.dampings_[i]); 
    }

    // publish state
    if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time)
    {
      if (state_pub_.trylock())
      {
        last_state_publish_time_ += state_publisher_period_;
        state_pub_.msg_.header.stamp = time;
        state_pub_.msg_.velocity = tmp_cmd_.velocities_; 
        state_pub_.msg_.stiffness = tmp_cmd_.stiffnesses_; 
        state_pub_.msg_.damping = tmp_cmd_.dampings_; 
        state_pub_.unlockAndPublish();
      }
    }
  }

  void MJVIC::callback(const iai_control_msgs::MultiJointVelocityImpedanceCommandConstPtr& msg)
  {
    ROS_WARN_STREAM("Received a command callback, ignoring.");
    time_buffer_.get(tmp_now_);
    watchdog_.update(tmp_now_);

    //TODO: implement me
  }

  void MJVIC::initStatePublisher(const ros::NodeHandle& nh)
  {
    state_pub_.init(nh, "state", 1);
    state_pub_.lock();
    state_pub_.msg_.joint_names = joint_names_;
    state_pub_.msg_.velocity.resize(joint_names_.size());
    state_pub_.msg_.stiffness.resize(joint_names_.size());
    state_pub_.msg_.damping.resize(joint_names_.size());
    state_pub_.unlock();
  }
}

PLUGINLIB_EXPORT_CLASS(iai_ros_controllers::MultiJointVelocityImpedanceController, controller_interface::ControllerBase)
