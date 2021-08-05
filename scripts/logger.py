#!/usr/bin/env python
import os
import json
import rospy
import unicodedata
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from loggerbib import Logger, LogWriter, ContextualLogger, LogDir, LogLevel
import roslib.message

log_file = None
log_path = ""
end_path = ""
simulation_init_time = 0.0
simulation_timeout_min = 15
simulation_timeout_s = simulation_timeout_min*60
END_KEYWORDS = ["ENDSIM","FAILURE","ENDLOWBATT","ENDTIMEOUTSIM"]
logwriter = None
logger = None

class RosTimer():
    def now(self):
        return rospy.get_time() - simulation_init_time

rostimer = RosTimer()

def check_timeout(event):
    global simulation_timeout_s, simulation_init_time, logger, end_path
    if (rospy.get_time() - simulation_init_time) > simulation_timeout_s:
        logwriterend = LogWriter(end_path)
        loggerend = Logger(logwriterend, rostimer)
        loggerend.log('ENDTIMEOUTSIM', 'logger', level=LogLevel.DEBUG)

def callback_log(data):
    global log_path, simulation_init_time, logger
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
        logger.log(json.dumps(logdata['content']), entity=logdata['entity'], level=level_map[logdata['level']])
    except Exception as e:
        rospy.logerr(e)
        logger.log(data.data)

def logger():
    rospy.init_node('logger', anonymous=True)
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
    robot_subs = []
    rospy.loginfo("Initializing simulation logger...")
    # hold node until /clock is initialized
    while rospy.get_time() == 0:
        rospy.logwarn("Waiting for clock...")
        rospy.sleep(0.1)
    
    rospy.Timer(rospy.Duration(1), check_timeout)
    simulation_init_time = rospy.get_time() # time in secs

    logger.log("ROBOTS_CONFIG="+os.environ['ROBOTS_CONFIG'], entity='logger', level=LogLevel.DEBUG)
    rospy.loginfo("ROBOTS_CONFIG="+os.environ['ROBOTS_CONFIG'])
    logger.log("NURSES_CONFIG="+os.environ['NURSES_CONFIG'], entity='logger', level=LogLevel.DEBUG)
    rospy.loginfo("NURSES_CONFIG="+os.environ['NURSES_CONFIG'])
    logger.log('Simulation open', entity='logger', level=LogLevel.DEBUG)
    rospy.loginfo("Simulation open")
    log_sub = rospy.Subscriber("/log", String, callback_log)
    logger.log('subcribing to in the topic /log', entity='logger', level=LogLevel.DEBUG)
    rospy.loginfo("subcribing to in the topic /log")

    rate = rospy.Rate(1.0/10.0) # 0.1 hz
    while not rospy.is_shutdown():
        logger.flush()
        rate.sleep()

    logger.log('end!')
    logger.flush()

if __name__ == '__main__':
    try:
        logger()
    except rospy.ROSInterruptException:
        logger.log('end!')
        logger.flush()
