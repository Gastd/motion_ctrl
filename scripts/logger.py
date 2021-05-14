#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

linear_sum = 0
n_linear = 0
angular_sum = 0
n_angular = 0
log_file = None
log_path = ""

def callback_log(data):
    global log_path
    with open(log_path, "a+") as myfile:
        rospy.loginfo(data.data)
        myfile.write(data.data)
        myfile.write('\n')
    # global linear_sum
    # global n_linear
    # global angular_sum
    # global n_angular
    # linear_vel = data.linear.x
    # angular_vel = data.angular.z

    # linear_sum = linear_sum + linear_vel
    # n_linear = n_linear + 1
    # angular_sum = angular_sum + angular_vel
    # n_angular = n_angular + 1

    # rospy.loginfo("ANGULAR: total = %.2f, avg_vel = %.2f, amostras = %d"%(angular_sum, (angular_sum/n_angular), n_angular))
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    global log_path

    rospy.loginfo("Setting logger...")
    current_path = os.getcwd() + "/logger_sim"
    try:
        os.mkdir(current_path)
    except OSError:
        print ("Creation of the directory %s failed" % current_path)
        rospy.loginfo ("Creation of the directory %s failed" % current_path)
    else:
        print ("Successfully created the directory %s" % current_path)
        rospy.loginfo ("Successfully created the directory %s" % current_path)
    log_path = current_path + "/experiment.log"
    # log_file = open(log_path, "a+")

    # open file to log all received data
    n_robots = int(os.environ['N_ROBOTS'])
    robot_subs = []
    rospy.loginfo("Initializing simulation logger...")
    with open(log_path, "a+") as myfile:
        for i in range(1, n_robots+1):
            robot_name = os.environ['ROBOT_NAME_'+str(i)]
            # robot_pose = os.environ['ROBOT_POSE_'+str(i)][1:-1].split(';')
            robot = rospy.Subscriber("/"+robot_name+"/log", String, callback_log)
            # robot.add_to_simulation(x=float(robot_pose[0]), y=float(robot_pose[1]))
            myfile.write("Subcribing to "+robot_name+" in the toppic "+"/"+robot_name+"/log")
            myfile.write('\n')
            rospy.loginfo(str(myfile))
            robot_subs.append(robot)

    nurse_sub = rospy.Subscriber("/nurse/log", String, callback_log)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()