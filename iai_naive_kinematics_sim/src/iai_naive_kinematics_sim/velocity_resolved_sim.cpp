#include <ros/ros.h>
#include <iai_naive_kinematics_sim/iai_naive_kinematics_sim.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class SimulatorNode
{
  public:
    SimulatorNode(const ros::NodeHandle& nh): 
      nh_(nh), sim_frequency_(0.0), sim_(iai_naive_kinematics_sim::System()) {}
    ~SimulatorNode() {}

    void init()
    {
      readSimFrequency();

      std::vector<std::string> controlled_joints =
        readControlledJoints();

      sim_.init(readUrdf(), controlled_joints, readWatchdogPeriod());
      sim_.setSubJointState(readStartConfig());

      startSubs(controlled_joints);
    }

    void run()
    {
      pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
      ros::Rate sim_rate(sim_frequency_);
      while(ros::ok())
      {
        sim_.update(ros::Time::now(), 1.0/sim_frequency_);
        pub_.publish(sim_.getJointState());
        ros::spinOnce();
        sim_rate.sleep();
      }
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    std::vector<ros::Subscriber> subs_;
    double sim_frequency_;
    iai_naive_kinematics_sim::System sim_;

    void callback(const std_msgs::Float64::ConstPtr& msg, const std::string& name)
    {
      sim_.setVelocityCommand(name, msg->data, ros::Time::now());
    }

    urdf::Model readUrdf() const
    {
      std::string urdf_descr_;
      assert(nh_.getParam("/robot_description", urdf_descr_));
      urdf::Model model;
      assert(model.initString(urdf_descr_));
      return model;
    }

    void readSimFrequency()
    {
      nh_.param("sim_frequency", sim_frequency_, 50.0);
      assert(sim_frequency_ > 0.0);
      ROS_INFO("sim_frequency: %f", sim_frequency_);
    }

    double readWatchdogPeriod() const
    {
      double watchdog_period;
      nh_.param("watchdog_period", watchdog_period, 0.1);
      assert(watchdog_period > 0.0);
      ROS_INFO("watchdog_period: %f", watchdog_period);

      return watchdog_period;
    }

    std::vector<std::string> readControlledJoints() const
    {
      std::vector<std::string> controlled_joints;
      controlled_joints.clear();
      assert(nh_.getParam("controlled_joints", controlled_joints));
      std::string out_string;
      for(size_t i =0; i < controlled_joints.size(); ++i)
        out_string += " " + controlled_joints[i];
      ROS_INFO("controlled joints:%s", out_string.c_str());

      return controlled_joints;
    }

    sensor_msgs::JointState readStartConfig() const
    {
      std::map<std::string, double> start_config;
      nh_.getParam("start_config", start_config);
      sensor_msgs::JointState joint_state;
      for(std::map<std::string, double>::const_iterator it=start_config.begin(); 
          it!=start_config.end(); ++it)
      {
        joint_state.name.push_back(it->first);
        joint_state.position.push_back(it->second);
        joint_state.velocity.push_back(0.0);
        joint_state.effort.push_back(0.0);
        ROS_INFO("start config for '%s': %f", it->first.c_str(), it->second);
      }

      return joint_state;
    }

    void startSubs(const std::vector<std::string>& controlled_joints)
    {
      for(size_t i=0; i<controlled_joints.size(); ++i)
      {
        boost::function<void(const std_msgs::Float64::ConstPtr&)> f =
          boost::bind(&SimulatorNode::callback, this, _1, controlled_joints[i]);
        std::string topic = "/" + controlled_joints[i] + "/vel_cmd";
        ros::Subscriber sub = nh_.subscribe(topic, 1, f);
        subs_.push_back(sub);
      }
    }
};

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"velocity_resolved_sim");

  SimulatorNode sim(ros::NodeHandle("~"));

  sim.init();
  sim.run();

  return 0;
}
