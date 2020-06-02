#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <iostream> 

double pick_up_goal[2] = {2, 3};
double drop_off_goal[2] = {13, 3.3};
double distance_threshold = 0.25;

bool pick_up = false;
bool drop_off = false;
bool at_pick_up = false;
bool at_drop_off = false;



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
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    return marker;
}

void changeColorShape(visualization_msgs::Marker& marker, uint32_t shape, char color){
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    
    if (color == 'r'){
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
    }
    else if (color == 'g'){
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
    }
    else if (color == 'b'){
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
    } 
    else{
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
    }  

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
    marker_pub.publish(marker);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //ROS_INFO("OdomCallback");
    float pose_x = msg->pose.pose.position.x;
    float pose_y = msg->pose.pose.position.y;
    // Position tolerance
    double distance;

    if (pick_up){
        ROS_INFO("Distance to pickup");
        distance = sqrt(pow(pose_x - pick_up_goal[0], 2) + pow(pose_y - pick_up_goal[1], 2));
        std::cout << distance << std::endl;
        if (distance < distance_threshold){
	  at_pick_up = true;
          pick_up = false;
	}
    }
    else if (drop_off){
        ROS_INFO("Distance to dropoff");
        distance = sqrt(pow(pose_x - drop_off_goal[0], 2) + pow(pose_y - drop_off_goal[1], 2));
        std::cout << distance << std::endl;
        if (distance < distance_threshold){
	  at_drop_off = true;
          drop_off = false;
        }
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 100, odomCallback);
  pick_up = true;
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  visualization_msgs::Marker marker = createMarker(shape);
  ROS_INFO("Pick up marker");
  setMarkersPose(marker, pick_up_goal[0], pick_up_goal[1], 0.0, 1.0);
    
  while (ros::ok())
  {
    ros::spinOnce();
    
    if (pick_up){
      publishMarker(marker, marker_pub);
    }
    else if (at_pick_up){
	  // Delete marker
          changeMarkerStatus(marker, false);
          publishMarker(marker, marker_pub);
          ROS_INFO("Object has been picked up, hidding marker");
          // Wait 5 seconds
          ros::Duration(5.0).sleep();
          drop_off = true;
          //Set drop off marker
          setMarkersPose(marker, drop_off_goal[0], drop_off_goal[1], 0.0, 1.0);
          changeMarkerStatus(marker, true);
          changeColorShape(marker, visualization_msgs::Marker::CUBE , 'r');
          publishMarker(marker, marker_pub);
          ROS_INFO("Drop off marker");
	  at_pick_up = false;

    }
    else if (at_drop_off){
          // Delete marker
          //changeMarkerStatus(marker, false);
          changeColorShape(marker, visualization_msgs::Marker::SPHERE , 'g');
          publishMarker(marker, marker_pub);
          drop_off = false;
          ROS_INFO("Object has been drop off, hidding marker");
          ROS_INFO("Tasks finished"); 
          ros::Duration(5.0).sleep();    
          return 0;
          }
   }
 }
