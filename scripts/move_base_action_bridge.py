#! /usr/bin/env python

import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
#include <move_base_msgs/MoveBaseAction.h>
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('turtlebot1/move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveBaseGoal()
    #we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = rospy.get_rostime();

    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    # Sends the goal to the action server.
    rospy.loginfo("Sending Goal")
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('move_base_py')
        result = move_client()
        rospy.loginfo(result)
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
        # print("program interrupted before completion", file=sys.stderr)