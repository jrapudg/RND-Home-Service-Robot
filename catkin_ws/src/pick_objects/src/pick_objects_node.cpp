#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal define_goal(double x, double y, double w){
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  return goal;
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

 
  move_base_msgs::MoveBaseGoal pick_up_goal = define_goal(2.0, 3.0, -1.7172);
  move_base_msgs::MoveBaseGoal drop_off_goal = define_goal(13.0, 3.3, 1.7172);
  

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick up goal");
  ac.sendGoal(pick_up_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot succesfully arrived to pick up goal");
  else
    ROS_INFO("Robot failed to move to pick up goal");

  ros::Duration(5.0).sleep();

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending drop off goal");
  ac.sendGoal(drop_off_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot succesfully arrived to drop off goal");
  else
    ROS_INFO("Robot failed to move to drop off goal");

  ros::Duration(5.0).sleep();

  return 0;
}
