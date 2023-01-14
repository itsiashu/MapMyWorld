#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "display_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "display_markers";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  // marker scale
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // marker positions
  double pickup_pos_x  = -1;
  double pickup_pos_y  =  3;
  double dropoff_pos_x = 0.5;
  double dropoff_pos_y = 0.5;

  // set blue color
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  
  ROS_INFO("publish marker ");
  marker_pub.publish(marker);
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pickup_pos_x;
  marker.pose.position.y = pickup_pos_y;
  ros::Duration(5.0).sleep();
  
  ROS_INFO("hide marker");
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ros::Duration(5.0).sleep();
  
  ROS_INFO("publish marker");
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = dropoff_pos_x;
  marker.pose.position.y = dropoff_pos_y;
  marker_pub.publish(marker);
  ros::Duration(5.0).sleep();
  ROS_INFO("Done!");
  return 0;
  r.sleep(); 
}
