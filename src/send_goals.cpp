#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//tell the action client that we want to spin a thread by default
MoveBaseClient *acp;
ros::Publisher feedback_pub;

void sendGoalToMoveBase(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = msg->pose;

  // goal.target_pose.pose.position.x = 1.0;
  // goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  acp->sendGoal(goal);

  acp->waitForResult();

  if(acp->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_msgs::String done;
    done.data = "Done";
    ROS_INFO_STREAM("Hooray, the base moved "<< msg->pose.position.x << " meter forward");
    feedback_pub.publish(done);
  }
  else
    ROS_INFO_STREAM("The base failed to move forward "<< msg->pose.position.x << " meter for some reason");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  MoveBaseClient ac("move_base", true);
  acp = &ac;

  ros::Subscriber sub = n.subscribe("send_goal", 1000, sendGoalToMoveBase);
  feedback_pub = n.advertise<std_msgs::String>("mb_feedback", 1000);

  //wait for the action server to come up
  while(!acp->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::spin();

  return 0;
}
