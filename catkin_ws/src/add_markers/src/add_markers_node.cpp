#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

double pick_up_goal[2] = {2, 3};
double drop_off_goal[2] = {13, 3.3};

void changeMarkerStatus(visualization_msgs::Marker& marker, bool flag){
    if(flag){
      marker.action = visualization_msgs::Marker::ADD;
    }
    else{
      marker.action = visualization_msgs::Marker::DELETE;
    }  
}

visualization_msgs::Marker createMarker(uint32_t shape){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    changeMarkerStatus(marker, true);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    return marker;
}

void setMarkersPose(visualization_msgs::Marker& marker, double x, double y, double angle_z, double angle_w){
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = angle_z;
    marker.pose.orientation.w = angle_w;
}

void publishMarker(visualization_msgs::Marker& marker, ros::Publisher& marker_pub){
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        throw "ROS error, it is not ok!";
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO("Pick up marker");
    marker_pub.publish(marker);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odomCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker = createMarker(shape);
    setMarkersPose(marker, pick_up_goal[0], pick_up_goal[1], 0.0, 1.0);
    
    //Publish marker
    publishMarker(marker, marker_pub);
    
    // Wait 5 seconds
    ros::Duration(5.0).sleep();
    
    // Delete marker
    changeMarkerStatus(marker, false);
    publishMarker(marker, marker_pub);
    ROS_INFO("Object has been picked up, hidding marker");

    // Wait 5 seconds
    ros::Duration(5.0).sleep();
   
    //Set drop off marker
    setMarkersPose(marker, drop_off_goal[0], drop_off_goal[1], 0.0, 1.0);
    changeMarkerStatus(marker, true);
    publishMarker(marker, marker_pub);
    ROS_INFO("Drop off marker");
    
    // Wait 5 seconds
    ros::Duration(5.0).sleep();

    ROS_INFO("Tasks finished");     
    //return 0;
    r.sleep();
   }
 }
