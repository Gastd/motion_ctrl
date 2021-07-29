#!/usr/bin/env python
import os
import json
import rospy
import unicodedata
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from loggerbib import Logger, LogWriter, ContextualLogger, LogDir, LogLevel
import roslib.message

linear_sum = 0
n_linear = 0
angular_sum = 0
n_angular = 0
log_file = None
log_path = ""
end_path = ""
simulation_init_time = 0.0
simulation_timeout_min = 15
simulation_timeout_s = simulation_timeout_min*60
END_KEYWORDS = ["ENDSIM","FAILURE","ENDLOWBATT","ENDTIMEOUTSIM"]

class RosTimer():
    def now(self):
        return rospy.get_time() - simulation_init_time

rostimer = RosTimer()
logwriter = None
logger = None

def formatlog(loginfo, severity):
    global simulation_init_time
    return (str(rospy.get_time() - simulation_init_time) + 
               ',['+severity+'],'+
               loginfo)

def check_timeout(event):
    global simulation_timeout_s, simulation_init_time, logger, end_path
    # end_path = current_path + '/trial.log'
    if (rospy.get_time() - simulation_init_time) > simulation_timeout_s:
        logwriterend = LogWriter(end_path)
        loggerend = Logger(logwriterend, rostimer)
        loggerend.log('ENDTIMEOUTSIM', 'logger', level=LogLevel.DEBUG)

def callback_log(data):
    global log_path, simulation_init_time, logger
    # add time since simulation init into the log
    # data.data = str(rospy.get_time() - simulation_init_time)+','+data.data
    rospy.loginfo(data.data)
    for keyword in END_KEYWORDS:
        if keyword in data.data:
            logwriterend = LogWriter(end_path)
            loggerend = Logger(logwriterend, rostimer)
            loggerend.log(keyword, 'logger', level=LogLevel.DEBUG)
            loggerend.flush()
            logger.flush()
            rospy.signal_shutdown('shutdown requested by keyword: {}'.format(keyword))

    try:
        logdata = json.loads(data.data)
        level_map = {
            'info': LogLevel.INFO,
            'debug': LogLevel.DEBUG
        }
        rospy.loginfo('content = {}'.format(logdata['content']))
        # content = unicodedata.normalize('NFKD', '{}'.format(logdata['content'])).encode('ascii', 'ignore')
        logger.log(json.dumps(logdata['content']), entity=logdata['entity'], level=level_map[logdata['level']])
    except Exception as e:
        rospy.logerr(e)
        logger.log(data.data)

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
    global log_path, simulation_init_time, logger, logwriter, end_path

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
    log_path = current_path + '/{:0>2d}_{}.log'.format(int(os.environ['TRIAL']), os.environ['TRIAL_CODE'])
    end_path = current_path + '/trial.log'
    logwriter = LogWriter(log_path)
    logger = Logger(logwriter, rostimer)

    # open file to log all received data
    # n_robots = int(os.environ['N_ROBOTS'])
    robot_subs = []
    nurse_sub = rospy.Subscriber("/log", String, callback_log)
    rospy.loginfo("Initializing simulation logger...")
    # hold node until /clock is initialized
    while rospy.get_time() == 0:
        rospy.logwarn("Waiting for clock...")
        rospy.sleep(0.1)
    
    rospy.Timer(rospy.Duration(1), check_timeout)
    simulation_init_time = rospy.get_time() # time in secs
    # data = String()
    # data.data = str(rospy.get_time() - simulation_init_time)+',[debug],logger,init,time'
    # callback_log(data)

    logger.log("ROBOTS_CONFIG="+os.environ['ROBOTS_CONFIG'], entity='logger', level=LogLevel.DEBUG)
    rospy.loginfo("ROBOTS_CONFIG="+os.environ['ROBOTS_CONFIG'])
    logger.log("NURSES_CONFIG="+os.environ['NURSES_CONFIG'], entity='logger', level=LogLevel.DEBUG)
    rospy.loginfo("NURSES_CONFIG="+os.environ['NURSES_CONFIG'])
    logger.log('Simulation open', entity='logger', level=LogLevel.DEBUG)
    rospy.loginfo("Simulation open")
    logger.log('subcribing to in the topic /log', entity='logger', level=LogLevel.DEBUG)
    rospy.loginfo("subcribing to in the topic /log")

    rate = rospy.Rate(1.0/10.0) # 0.1 hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        logger.flush()
        rate.sleep()

    logger.log('end!')
    logger.flush()

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    listener()
