#! /usr/bin/python

import rospy
from sensor_msgs.msg import *

rospy.init_node('scan_gmapping')

pub = rospy.Publisher('scan_gmapping', LaserScan, queue_size=1)

def callback(data):
    msg = data
    msg.angle_max = -msg.angle_min
    pub.publish(msg)

rospy.Subscriber("scan", LaserScan, callback)
rospy.spin()
