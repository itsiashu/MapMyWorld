#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// Consider Robots movements as States
enum States {
    Pickup,
    Moving,
    Dropoff,
};

// Define parameters
// pickup pos
double pickup_pos_x  = -1;
double pickup_pos_y  =  3.0;
// dropoff pos
double dropoff_pos_x = 0.5;
double dropoff_pos_y = 0.5;
// current robot poses
double robot_cur_x; 
double robot_cur_y;
// how close robot is to pickup or dropoff
double closeness = 0.005;
double pickup_diff;
double dropoff_diff;

// callback to send robot poses
void robot_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
{
  robot_cur_x = amcl_msg->pose.pose.position.x;
  robot_cur_y = amcl_msg->pose.pose.position.y;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    States state = Pickup;
    //std::string state = "";

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //marker.pose.position.x = 0;
    //marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Subscribe to amcl pose
    ros::Subscriber sub1 = n.subscribe("/amcl_pose", 1000, robot_pose_cb);

    // wait times (secs)
    int total_wait = 5;
    int cur_wait = 0;
    while (ros::ok())
    {
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
        if (!ros::ok())
        {
            return 0;
        }
        //ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
        }
        marker_pub.publish(marker);

        // Cycle between different shapes
        /*
        switch (shape)
        {
        case visualization_msgs::Marker::CUBE:
        shape = visualization_msgs::Marker::SPHERE;
        break;
        case visualization_msgs::Marker::SPHERE:
        shape = visualization_msgs::Marker::ARROW;
        break;
        case visualization_msgs::Marker::ARROW:
        shape = visualization_msgs::Marker::CYLINDER;
        break;
        case visualization_msgs::Marker::CYLINDER:
        shape = visualization_msgs::Marker::CUBE;
        break;
        } */
        //ros::Duration(5).sleep(); // pause 5 seconds
        //r.sleep();
        switch (state) {
            case Pickup:
                // Check pickup
                pickup_diff = abs(robot_cur_x - pickup_pos_x) + abs(robot_cur_y - pickup_pos_y);
                if (pickup_diff >= closeness) {
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = pickup_pos_x;
                    marker.pose.position.y = pickup_pos_y;
                }
                else {
                    state = Moving;
                    marker.action = visualization_msgs::Marker::DELETE;
                }
                break;
            case Moving:
                if (cur_wait  <= total_wait)
                    cur_wait  += 1;
                else
                    state = Dropoff;
                break;
            case Dropoff:
                // check dropoff
                dropoff_diff = abs(robot_cur_x - dropoff_pos_x) + abs(robot_cur_y - dropoff_pos_y);
                if (dropoff_diff > closeness)
                    marker.action = visualization_msgs::Marker::DELETE;
                else {
                    sleep(2);
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = dropoff_pos_x;
                    marker.pose.position.y = dropoff_pos_y;
                }
                break;
  	    }
        marker_pub.publish(marker);
        sleep(1);
        ros::spinOnce();
    }
}