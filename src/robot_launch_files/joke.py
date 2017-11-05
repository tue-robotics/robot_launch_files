#! /usr/bin/python

import rospy, sys, random, time
from geometry_msgs.msg import Twist
from smach_msgs.msg import SmachContainerStatus

rospy.init_node('joke')

robot_name = rospy.get_namespace().split("/")[-2]
if robot_name == "amigo":
    from robot_skills.amigo import Amigo
    robot = Amigo()
elif robot_name == "sergio":
    from robot_skills.sergio import Sergio
    robot = Sergio()
else:
    rospy.loginfo("Unknown robot namespace %s" % robot_name)
    sys.exit(1)

hmi = robot.hmi
s = robot.speech

last_update = rospy.Time.now()
minutes = 20

from collections import deque

jokes = [
    "What do you call a fish with no eyes? A fsh.",
    "You don't need a parachute to go skydiving. You need a parachute to go skydiving twice.",
    "What is the difference between a snowman and a snowwomen? Snowballs.",
    "What is Bruce Lee's favorite drink? Wataaaaah!",
    "A blind man walks into a bar. And a table. And a chair.",
    "Its color is yellow and when you push the button, it turns red?         A chick in the blender",
    "Why was 6 afraid of 7?                  Because 7, 8, 9",
    "How do you call a robot that always goes the long way around?                       R 2 detour",
    "Can a kangaroo jump higher than a house?                  Of course, a house does not jump at all.",
    "What should you put on the tomb stone of a mathematician?              He did not count with this...",
    "Why do cows wear bells?               Their horns do not work.",
    "Why did the physics teacher break up with the biology teacher?            There was no chemistry."
]
jokes = deque(jokes)
random.shuffle(jokes)

def joke():
    global last_update
    global minutes
    global jokes

    jokes.rotate()

    s.speak(jokes[0])

    time.sleep(5.0)

    s.speak("Would you like to hear another one?")
    r = None

    r = hmi.query('', 'T -> yes | no', 'T').sentence

    if not r or r == "no" or r.result == "":
        s.speak("Ok, I will be quiet for another %d minutes" % minutes)
        last_update = rospy.Time.now()

def timer_callback(event):
    global last_update

    if (rospy.Time.now() - last_update).to_sec() > minutes * 60:
        try:
            joke()
        except:
            last_update = rospy.Time.now()

def callback(data):
    global last_update
    last_update = rospy.Time.now()

rospy.Subscriber("base/references", Twist, callback)
rospy.Subscriber("smach/container_status", SmachContainerStatus, callback)
rospy.Timer(rospy.Duration(2), timer_callback)
rospy.spin()
