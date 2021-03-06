#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_goals");

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "wamv/base_link";                                                                                     
  goal.target_pose.header.stamp = ros::Time::now();                                                                                    
                                                                                                                                       
  goal.target_pose.pose.position.x = 45.359;                                                                                          
  goal.target_pose.pose.position.y = 9.301;                                                                                          
  goal.target_pose.pose.orientation.w = 0.8;                                                                                       
                                                                                                                                       
  ROS_INFO("Sending goal");                                                                                                            
  ac.sendGoal(goal);                                                                                                                   
                                                                                                                                       
  ac.waitForResult();                                                                                                                  
                                                                                                                                       
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)                                                                     
    ROS_INFO("Boom, Johnny 5 moved to goal 1");                                                                                        
  else                                                                                                                                 
    ROS_INFO("Johnny 5 is stuck");

  // Move to next point 
  goal.target_pose.header.frame_id = "wamv/odom";                                                                                          
  goal.target_pose.header.stamp = ros::Time::now();                                                                                    
                                                                                                                                       
  goal.target_pose.pose.position.x = 1.1;
  goal.target_pose.pose.position.y = 0.1;
  goal.target_pose.pose.orientation.w = 0.1;
                                                                                                                                       
  ROS_INFO("Sending goal");                                                                                                            
  ac.sendGoal(goal);                                                                                                                   
                                                                                                                                       
  ac.waitForResult();                                                                                                                  
                                                                                                                                       
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)                                                                     
    ROS_INFO("Boom, Johnny 5 moved to goal 2");                                                                                        
  else                                                                                                                                 
    ROS_INFO("Johnny 5 is stuck");


  return 0;
}
