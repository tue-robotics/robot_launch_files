#!/usr/bin/python

"""Bodypart resetter

Usage:
  bodypart_resetter <robot> <bodypart> <timeout>

Examples:
  bodypart_resetter sergio base 2
  bodypart_resetter amigo base 10
"""

import rospy
import sys
import time
from diagnostic_msgs.msg import DiagnosticArray as Status
from std_msgs.msg import UInt8MultiArray as Command
from docopt import docopt

def callback(data):
    global pub, bodypart, timeout, pub_topic
    index = 0
    for status in data.status:
        if status.name == bodypart and status.level == 4:
            rospy.logwarn("%s error detected", bodypart)
            time.sleep(timeout)
            pub.publish(Command(data=[index, 24]))
            rospy.loginfo("Sending reset command to topic %s for bodypart %s", pub_topic, bodypart)
            time.sleep(1)
            pub.publish(Command(data=[index, 22]))
            rospy.loginfo("Sending start command to topic %s for bodypart %s", pub_topic, bodypart)
            sys.exit()
        index += 1

if __name__ == '__main__':

    sys.argv = [ v for v in sys.argv if v[0] != "_" ]

    arguments = docopt(__doc__)

    sub_topic = "/%s/hardware_status"%arguments["<robot>"]
    pub_topic = "/%s/dashboard_ctrlcmds"%arguments["<robot>"]
    bodypart = arguments["<bodypart>"]
    timeout = int(arguments["<timeout>"])

    rospy.init_node("bodypart_resetter_%s_%s"%(arguments["<robot>"], bodypart))

    rospy.loginfo("Initialized bodypart resetter that listens to %s and publishes to %s with timeout %d for bodypart %s", sub_topic, pub_topic, timeout, bodypart)

    pub = rospy.Publisher(pub_topic, Command, queue_size=1)
    rospy.Subscriber(sub_topic, Status, callback, queue_size=1)

    rospy.spin()
