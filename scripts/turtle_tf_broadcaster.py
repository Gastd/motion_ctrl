#!/usr/bin/env python  

import roslib
import rospy

import tf
# from tf import listener
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


pub = None
def handle_turtle_pose(msg, turtlename):
    new_msg = Odometry()
    new_msg.header = msg.header
    new_msg.header.frame_id = "map"
    new_msg.header.stamp = rospy.get_rostime()
    new_msg.child_frame_id = "base_footprint"
    new_msg.pose.pose = msg.pose
    pub.publish(new_msg)
    
def handle_turtle_odom(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)
    # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    # t.transform.rotation.x = q[0]
    # t.transform.rotation.y = q[1]
    # t.transform.rotation.z = q[2]
    # t.transform.rotation.w = q[3]

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    # listener = tf.TransformListener()
    # br = tf.TransformBroadcaster()
    # turtlename = 'turtlebot1'
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename, PoseStamped, handle_turtle_pose, turtlename)
    rospy.Subscriber('/%s/odom' % turtlename, Odometry, handle_turtle_odom, turtlename)
    pub = rospy.Publisher('base_pose_ground_truth', Odometry, queue_size=1)

    rospy.spin()
