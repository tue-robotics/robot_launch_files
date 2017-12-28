#! /usr/bin/python

# from __future__ import print_function

import os
import socket
import time
import csv
import rospy
from math import sqrt, pow, pi
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import Odometry

DEFAULT_LOCATION = "~/MEGA/Odometer"
FILENAME = "odometer.csv"
ROUND_LEVEL = 5


class Odometer(object):
    def __init__(self):
        hostname = socket.gethostname()
        date = time.strftime("%Y_%m_%d")

        hostfolderpath = os.path.join(os.path.expanduser(DEFAULT_LOCATION), hostname)
        datefolderpath = os.path.join(hostfolderpath, date)
        newfilepath = os.path.join(datefolderpath, FILENAME)
        lastfilepath = ""

        self.file_has_header = False

        self.total_time = 0
        self.total_distance = 0
        self.total_rotation = 0

        self.data = []  # list of dicts with data
        self.last_pose = None
        self.last_time = rospy.Time.now().secs

        # Check if there exist a previous file to read from
        if os.path.exists(hostfolderpath):
            if os.path.exists(newfilepath):
                lastfilepath = newfilepath
                rospy.logdebug("Last data file is today's file")
            else:
                dirs = [item for item in sorted(os.listdir(hostfolderpath), reverse=True) if
                        os.path.isdir(os.path.join(hostfolderpath, item))]
                if dirs:
                    for dir in dirs:
                        filepath = os.path.join(hostfolderpath, dir, FILENAME)
                        if os.path.exists(filepath):
                            lastfilepath = filepath
                            rospy.logdebug("Found last file: {}".format(lastfilepath))
                            break
                    if not lastfilepath:
                        rospy.logdebug("Not found a file in the folders: {}".format(dirs))
                else:
                    rospy.logdebug("No folders with possible data files there yet")
        else:
            os.makedirs(hostfolderpath)
            rospy.logdebug("No folder for hostname: '{}' found".format(hostname))

        # look for data in last data file, if found
        if not lastfilepath:
            rospy.logdebug("No previous data found. Starting from zero")
        else:
            rospy.logdebug("Reading from last data file: {}".format(lastfilepath))
            with open(lastfilepath, "r") as f:
                reader = csv.reader(f)
                # in case an empty file is there. The first line will stay empty. Therefore the header needs to be found
                found_header = False
                for header in reader:
                    if header == ['timestamp', 'distance', 'rotation', 'time']:
                        found_header = True
                        break

                if found_header:
                    # Going over all lines.(this is the only option to get the last line) Doing nothing with other lines
                    for last_row in reader:
                        pass
                    if last_row:
                        last_row = dict(zip(header, last_row))
                        try:
                            self.total_distance = float(last_row['distance'])
                            self.total_rotation = float(last_row['rotation'])
                            self.total_time = int(float(last_row['time']))
                            if lastfilepath == newfilepath:
                                self.file_has_header = True
                            rospy.logdebug("Loaded data from file: {}".format(lastfilepath))
                        except Exception as e:
                            rospy.logerr(e)
                            rospy.signal_shutdown("Unable to read last data. Last data is corrupt")
                else:
                    rospy.logerr("No header found in file")

        # Create today's file if not already there
        if os.path.exists(newfilepath):
            self.new_file = open(newfilepath, "a")
            rospy.logdebug("Today's file already exists")
        else:
            if not os.path.exists(datefolderpath):
                os.makedirs(datefolderpath)
                rospy.logdebug("Folder of today doesn't exist yet")
            self.new_file = open(newfilepath, "w+")

        rospy.Subscriber("base/measurements", Odometry, self.callback)
        rospy.on_shutdown(lambda: self.shutdown())

    def sample(self):
        new_time = rospy.Time.now().secs
        time_delta = new_time - self.last_time
        self.total_time += time_delta
        self.last_time = new_time

        timestamp = time.strftime("%Y_%m_%d_%H_%M_%S")
        dist = round(self.total_distance, ROUND_LEVEL)
        rot = round(self.total_rotation, ROUND_LEVEL)
        t = self.total_time
        self.data.append({'timestamp': timestamp, 'distance': dist, 'rotation': rot, 'time': t})

    def write(self):
        writer = csv.DictWriter(self.new_file, fieldnames=['timestamp', 'distance', 'rotation', 'time'])
        if not self.file_has_header:
            rospy.logdebug("Printing header of csv file")
            writer.writeheader()
            self.file_has_header = True
        if self.data:
            rospy.logdebug("Writing data to csv file")
            writer.writerows(self.data)
            self.data = []

    def callback(self, msg):
        if self.last_pose:
            new_pose = msg.pose.pose
            pos = new_pose.position
            pos_old = self.last_pose.position
            distance_delta = sqrt(pow(pos.x-pos_old.x, 2) + pow(pos.y-pos_old.y, 2))
            if distance_delta < 0.5:  # If delta is too big, it is incorrect. Not doing anything with this data
                self.total_distance += distance_delta

                new_orientation = new_pose.orientation
                old_orientation = self.last_pose.orientation
                new_rotation = euler_from_quaternion([new_orientation.x,
                                                      new_orientation.y,
                                                      new_orientation.z,
                                                      new_orientation.w])
                old_rotation = euler_from_quaternion([old_orientation.x,
                                                      old_orientation.y,
                                                      old_orientation.z,
                                                      old_orientation.w])
                rotation_delta = new_rotation[2] - old_rotation[2]
                if rotation_delta >= pi:
                    rotation_delta -= 2*pi
                elif rotation_delta <= -pi:
                    rotation_delta += 2*pi
                self.total_rotation += abs(rotation_delta)

        self.last_pose = msg.pose.pose

    def activate_write(self, length):
        if len(self.data) >= length:
            self.write()

    def shutdown(self):
        self.sample()
        self.write()
        self.new_file.close()


if __name__ == '__main__':
    rospy.init_node("Odometer")

    r = float(rospy.get_param("~rate", float(1/60.0)))
    length = rospy.get_param("~data_length", 50)

    # robot_name = rospy.get_namespace().split("/")[-2]
    meter = Odometer()
    rate = rospy.Rate(max(r, 1e-3))

    while not rospy.is_shutdown():
        meter.sample()
        meter.activate_write(length)
        rate.sleep()
