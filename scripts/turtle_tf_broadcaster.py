#!/usr/bin/env python  

import roslib
import rospy

import tf
# from tf import listener
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros


pub = None
def handle_turtle_pose(msg, turtlename):
    new_msg = Odometry()
    new_msg.header = msg.header
    new_msg.header.frame_id = "map"
    new_msg.header.stamp = rospy.get_rostime()
    new_msg.child_frame_id = "base_footprint"
    new_msg.pose.pose = msg.pose
    pub.publish(new_msg)
    

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    # listener = tf.TransformListener()
    # br = tf.TransformBroadcaster()
    turtlename = 'turtlebot1'
    # turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename, PoseStamped, handle_turtle_pose, turtlename)
    pub = rospy.Publisher('base_pose_ground_truth', Odometry, queue_size=1)

    rospy.spin()
