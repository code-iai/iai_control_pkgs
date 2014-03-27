#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <iai_tf_relay_msgs/BroadcastTfAction.h>
#include <tf/transform_broadcaster.h>
#include <vector>

class TfRelay
{
  public:
    TfRelay(const ros::NodeHandle& nh) : 
        nh_(nh), action_server_(nh, "broadcast", false)
    {
      action_server_.registerGoalCallback( boost::bind(
          &TfRelay::commandGoalCallback, this ) );
      action_server_.start();
    }
  
    ~TfRelay() {}

  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<iai_tf_relay_msgs::BroadcastTfAction>
        action_server_;
    tf::TransformBroadcaster broadcaster_;

   void commandGoalCallback() throw ()
   {
     iai_tf_relay_msgs::BroadcastTfGoalConstPtr goal =
         action_server_.acceptNewGoal();

     ros::Rate frequency(goal->frequency);

     while(ros::ok())
     {
       if(action_server_.isPreemptRequested())
       {
         action_server_.setPreempted();
         return;  
       }

       broadcaster_.sendTransform(updateTimestamps(goal->transforms));
       frequency.sleep();
     }
   }

   std::vector<geometry_msgs::TransformStamped> updateTimestamps(
       const std::vector<geometry_msgs::TransformStamped>& transforms)
   {
     std::vector<geometry_msgs::TransformStamped> result;
     for(unsigned int i=0; i<transforms.size(); i++)
     {
       result.push_back(transforms[i]);
       result[i].header.stamp = ros::Time::now();
     }
     
     return result;
   }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_relay");

  ros::NodeHandle nh("~");

  TfRelay my_relay(nh);  
  
  ros::spin();

  return 0;
}
