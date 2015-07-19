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
from diagnostic_msgs.msg import DiagnosticArray as Status
from std_msgs.msg import UInt8MultiArray as Command
from docopt import docopt

def callback(data):
    global pub, bodypart, timeout
    index = 0
    for status in data.status:
        index += 1
        if status.name == bodypart and status.level == 4:
            time.sleep(timeout)
            pub.publish(Command([index, 4]))

if __name__ == '__main__':

    sys.argv = [ v for v in sys.argv if v[0] != "_" ]

    arguments = docopt(__doc__)

    sub_topic = "/%s/hardware_status"%arguments["<robot>"]
    pub_topic = "/%s/dashboard_ctrlcmds"%arguments["<robot>"]
    bodypart = arguments["<bodypart>"]
    timeout = int(arguments["<timeout>"])

    print "Initialized bodypart resetter that listens to %s and publishes to %s with timeout %d for bodypart %s"%(sub_topic, pub_topic, timeout, bodypart) 

    rospy.init_node("bodypart_resetter_%s_%s"%(arguments["<robot>"], bodypart))
    pub = rospy.Publisher(pub_topic, Command, queue_size=1)
    rospy.Subscriber(sub_topic, Status, callback)

    rospy.spin()
