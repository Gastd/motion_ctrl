#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

linear_sum = 0
n_linear = 0
angular_sum = 0
n_angular = 0

def callback(data):
    global linear_sum
    global n_linear
    global angular_sum
    global n_angular
    linear_vel = data.linear.x
    angular_vel = data.angular.z

    linear_sum = linear_sum + linear_vel
    n_linear = n_linear + 1
    angular_sum = angular_sum + angular_vel
    n_angular = n_angular + 1

    rospy.loginfo("LINEAR: total = %.2f, avg_vel = %.2f, amostras = %d"%(linear_sum, (linear_sum/n_linear), n_linear))
    rospy.loginfo("ANGULAR: total = %.2f, avg_vel = %.2f, amostras = %d"%(angular_sum, (angular_sum/n_angular), n_angular))
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/turtlebot1/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()