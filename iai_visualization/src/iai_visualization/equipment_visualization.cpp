#include <ros/ros.h>
#include <saphari_msgs/Equipment.h>
#include <saphari_msgs/PerceivedEquipment.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

class EquipmentVisualization
{
  public:
    EquipmentVisualization(const ros::NodeHandle& nh): nh_(nh)
    {
      publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("out_topic", 1);
      subscriber_ = nh_.subscribe("in_topic", 1, &EquipmentVisualization::callback, this);

      equipment_Map_[saphari_msgs::Equipment::BOWL] = "bowl";
      equipment_Map_[saphari_msgs::Equipment::CLAMP_BIG] = "clamp_big";
      equipment_Map_[saphari_msgs::Equipment::CLAMP_SMALL] = "clamp_small";
      equipment_Map_[saphari_msgs::Equipment::SCALPEL] = "scalpel";
      equipment_Map_[saphari_msgs::Equipment::SCISSORS] = "scissors";
    }

    ~EquipmentVisualization() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::map<uint8_t,std::string> equipment_Map_;

    void callback(const saphari_msgs::PerceivedEquipment::ConstPtr& msg)
    {
      saphari_msgs::PerceivedEquipment equipments = *msg;
      std::vector<visualization_msgs::Marker> markers;
      visualization_msgs::Marker marker;
      saphari_msgs::Equipment equipment;

      // Make the markers for the equipments
      for (std::vector<saphari_msgs::Equipment>::iterator it = equipments.perceived.begin(); it != equipments.perceived.end(); it++)
      {
        equipment = *it;
        marker = visualize_equipment(equipment);
        markers.push_back(marker);
        markers.push_back(visualize_equipment_label(marker, equipment_Map_[equipment.ID]));
      }

      visualization_msgs::MarkerArray markerArray;
      markerArray.markers = markers;
      publisher_.publish(markerArray);
    }

    visualization_msgs::Marker visualize_equipment(const saphari_msgs::Equipment& equipment)
    {
      visualization_msgs::Marker marker;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = equipment.pose.header.frame_id;
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "equipment";
      marker.id = equipment.ID;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::ARROW;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = equipment.pose.pose.position.x;
      marker.pose.position.y = equipment.pose.pose.position.y;
      marker.pose.position.z = equipment.pose.pose.position.z;
      marker.pose.orientation.x = equipment.pose.pose.orientation.x;
      marker.pose.orientation.y = equipment.pose.pose.orientation.y;
      marker.pose.orientation.z = equipment.pose.pose.orientation.z;
      marker.pose.orientation.w = equipment.pose.pose.orientation.w;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      return marker;
    }

    visualization_msgs::Marker visualize_equipment_label(visualization_msgs::Marker& equipment_marker, std::string label)
    {
      visualization_msgs::Marker marker(equipment_marker);

      marker.id = marker.id + 1000;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.z = marker.pose.position.z - marker.scale.z + 0.1;

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
  ros::init(argc, argv, "equipment_visualization");

  ros::NodeHandle nh("~");

  EquipmentVisualization equipment_visualization(nh);

  ros::spin();

  return 0;
}
