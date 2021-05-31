#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//tell the action client that we want to spin a thread by default
MoveBaseClient *acp;
ros::Publisher feedback_pub;
ros::Publisher log_pub;

// def formatlog(severity, who, loginfo, skill, params):
//     global simulation_init_time
//     return ('['+severity+'],'+
//                who+','+
//                loginfo+','+
//                skill+','+
//                params)

std::string formatlog(std::string severity,
                      std::string who,
                      std::string loginfo,
                      std::string skill,
                      std::string params)
{
  return (std::string("[")+severity+std::string("],")+
          who+std::string(",")+
          loginfo+std::string(",")+
          skill+std::string(",")+
          params);
}

void sendGoalToMoveBase(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  if (msg->header.frame_id == "base_link")
    return;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = msg->pose;

  // goal.target_pose.pose.position.x = 1.0;
  // goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  acp->sendGoal(goal);

  acp->waitForResult();

  std_msgs::String log_str;
  std::stringstream ssSkill, ssParams;
  if(acp->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_msgs::String done;
    done.data = "Done";
    ROS_INFO_STREAM("Hooray, the base moved "<< msg->pose.position.x << "," << msg->pose.position.y << " meter forward");
    ssSkill <<"Hooray, the base moved arrived";
    ssParams << "(x="<< msg->pose.position.x << ";y=" << msg->pose.position.y << ")";
    // log_str.data = std::getenv("ROBOT_NAME") + std::string(",") +std::to_string(ros::Time::now().toSec())+std::string(",robot arrived at (")+std::to_string(msg->pose.position.x)+std::string(";")+std::to_string(msg->pose.position.y)+std::string(")");
    log_str.data = formatlog("debug", std::getenv("ROBOT_NAME"), "move-base-info", ssSkill.str(), ssParams.str());
    log_pub.publish(log_str);
    feedback_pub.publish(done);
  }
  else
  {
    std_msgs::String done;
    done.data = "Fail";
    ROS_INFO_STREAM("The base failed to move forward "<< msg->pose.position.x << "," << msg->pose.position.y << " meter for some reason");
    // log_str.data = std::getenv("ROBOT_NAME") + std::string(",") +std::to_string(ros::Time::now().toSec())+std::string(",robot arrived at (")+std::to_string(msg->pose.position.x)+std::string(";")+std::to_string(msg->pose.position.y)+std::string(")");
    ssSkill <<"The base failed to move ";
    ssParams << "(x="<< msg->pose.position.x << ";y=" << msg->pose.position.y << ")";
    log_str.data = formatlog("debug", std::getenv("ROBOT_NAME"), "move-base-info", ssSkill.str(), ssParams.str());
    log_pub.publish(log_str);
    feedback_pub.publish(done);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  MoveBaseClient ac("move_base", true);
  acp = &ac;

  ros::Subscriber sub = n.subscribe("send_goal", 1000, sendGoalToMoveBase);
  feedback_pub = n.advertise<std_msgs::String>("mb_feedback", 1000);
  log_pub = n.advertise<std_msgs::String>("/log", 1000, true);

  //wait for the action server to come up
  std_msgs::String log_str;
  while(!acp->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    // log_str.data = std::getenv("ROBOT_NAME") + std::string(",") +std::to_string(ros::Time::now().toSec())+std::string(",Waiting for the move_base action server to come up");
    log_str.data = formatlog("debug", std::getenv("ROBOT_NAME"), "move-base-info", "Waiting for the move_base action server to come up", "");
    log_pub.publish(log_str);
  }

  // log_str.data = std::getenv("ROBOT_NAME") + std::string(",") +std::to_string(ros::Time::now().toSec())+std::string(",Move_base is up and ok....");
  log_str.data = formatlog("debug", std::getenv("ROBOT_NAME"), "move-base-info", "Move_base is up and ok....", "");
  log_pub.publish(log_str);
  // log_str.data = std::getenv("ROBOT_NAME") + std::string(",") +std::to_string(ros::Time::now().toSec())+std::string(",STARTSIM");
  // log_pub.publish(log_str);
  // log_str.data = std::to_string(ros::Time::now().toSec());
  // log_pub.publish(log_str);

  ros::spin();

  return 0;
}
