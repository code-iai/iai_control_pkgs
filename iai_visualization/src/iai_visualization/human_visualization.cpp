#include <ros/ros.h>
#include <saphari_msgs/Human.h>
#include <saphari_msgs/BodyPart.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

class HumanVisualization
{
  public:
    HumanVisualization(const ros::NodeHandle& nh): nh_(nh)
    {
      publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &HumanVisualization::callback, this);

      overwrite_frame_id_ = 0;
      if(nh_.getParam("human_frame_id", frame_id_))
        overwrite_frame_id_ = 1;

      bodypart_Map_[saphari_msgs::BodyPart::LEFTFOOT] = "left_foot";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTLEG] = "left_leg";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTKNEE] = "left_knee";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTTHIGH] = "left_high";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTFOOT] = "right_foot";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTLEG] = "right_leg";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTKNEE] = "right_knee";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTTHIGH] = "right_high";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTHIP] = "right_hip";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTHIP] = "left_hip";
      bodypart_Map_[saphari_msgs::BodyPart::NECK] = "neck";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTARM] = "right_arm";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTELBOW] = "right_elbow";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTFOREARM] = "right_forearm";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTHAND] = "right_hand";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTARM] = "left_arm";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTELBOW] = "left_elbow";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTFOREARM] = "left_forearm";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTHAND] = "left_hand";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTBOTTOMFACE] = "left_bottomface"; 
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTBOTTOMFACE] = "right_bottomface";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTTOPFACE] = "left_topface";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTTOPFACE] = "right_topface";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTCHEST] = "right_chest";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTCHEST] = "left_chest";

      bodypart_Map_[saphari_msgs::BodyPart::HEAD] = "head";
      bodypart_Map_[saphari_msgs::BodyPart::TORSO] = "torso";
      bodypart_Map_[saphari_msgs::BodyPart::RIGHTSHOULDER] = "right_shoulder";
      bodypart_Map_[saphari_msgs::BodyPart::LEFTSHOULDER] = "left_shoulder";
    }

    ~HumanVisualization() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::map<uint8_t,std::string> bodypart_Map_;
    std::string frame_id_;
    int overwrite_frame_id_;

    void callback(const saphari_msgs::Human::ConstPtr& msg)
    {
      saphari_msgs::Human human = *msg;
      std::vector<visualization_msgs::Marker> markers;
      visualization_msgs::Marker marker;
      saphari_msgs::BodyPart bodypart;

      // Make the markers for the bodyparts
      for (std::vector<saphari_msgs::BodyPart>::iterator it = human.bodyParts.begin(); it != human.bodyParts.end(); it++)
      {
        bodypart = *it;
        marker = visualize_bodypart(bodypart, human.header);
        markers.push_back(marker);
        markers.push_back(visualize_bodypart_label(marker, bodypart_Map_[bodypart.label]));
      }

      visualization_msgs::MarkerArray markerArray;
      markerArray.markers = markers;
      publisher_.publish(markerArray);
    }

    visualization_msgs::Marker visualize_bodypart(const saphari_msgs::BodyPart& bodypart, const std_msgs::Header& header)
    {
      visualization_msgs::Marker marker;

      // Set the frame ID and timestamp.
      if(overwrite_frame_id_)
        marker.header.frame_id = frame_id_;
      else
        marker.header.frame_id = header.frame_id;

      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "human_bodyparts";
      marker.id = bodypart.id;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::SPHERE;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = bodypart.centroid.x;
      marker.pose.position.y = bodypart.centroid.y;
      marker.pose.position.z = bodypart.centroid.z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = bodypart.radius;
      marker.scale.y = bodypart.radius;
      marker.scale.z = bodypart.radius;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      return marker;
    }

    visualization_msgs::Marker visualize_bodypart_label(visualization_msgs::Marker& bodypart_marker, std::string label)
    {
      visualization_msgs::Marker marker(bodypart_marker);

      marker.id = marker.id + 1000;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.z = marker.pose.position.z - marker.scale.z + 0.05;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      marker.text = label;

      return marker;
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_visualization");

  ros::NodeHandle nh("~");

  HumanVisualization human_visualization(nh);

  ros::spin();

  return 0;
}
