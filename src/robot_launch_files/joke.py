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

e = robot.ears
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
    "It's color is yellow and when you push the button, it turns red?         A chick in the blender",
    "Why was 6 afraid of 7?                  Because 7, 8, 9",
    "What happens if you want to join a list but there is a None in it?                  sequence item x: expected string, NoneType found",
    "Why was the robot angry?           Because someone kept pushing his buttons!",  # Source: http://boyslife.org/about-scouts/merit-badge-resources/robotics/19223/robot-jokes/
    "Why did the robot go back to robot school?        Because his skills were getting a little rusty!",
    "What do you get when you cross a robot and a tractor?         A trans-farmer!"
    "Are those real or were you upgraded in silicone valley?",
    # Source: http://www.jokes4us.com/pickuplines/robotpickuplines.html
    "Hey baby, want to grab some Java?",
    "Hey baby, what's your OS?",
    "Did you just break one of Asimov's Three Laws?          Because you've got 'fine' written all over you. ",
    "I'm going to void your warranty! My docking station or yours?",
    "Yes, I know you're metric- but I'm willing to convert.",
    "I support portrait and landscape modes. ",
    "Is it hot in here, or did your internal fan system just crash?",
    "If I were a function, would you call me?",
    "It that a joystick you're holding or are you just happy to see me?",
    "roses are #FF0000, violets are #0000FF, all my base, belong to you. ",
    "Is that a mirror in your anodized Titanium exterior plating?      Because I can see myself in your service port. ",
    "Is 2GB really your maximum RAM capacity? I heard otherwise...",
    "I hope you have an accellerometer, because I'm gonna rock your world. ",
    "Rusting is red, and my chipset's blue. Will you let me assimilate you?",
    "Your lips say 0 but your eyes say 1",
    "I've got a case of WD-40 in the back, wanna get drunk?",
    "Would you like a demo of my multitouch capabilities?",
    "Was your father a thief? Because he stole some titanium bolts and put them in your eyes. ",
    "Do you believe in love at first optical recognition, or should I ambulate by your location again?",
    "Wanna be debugged?",
    "You must be tired because you've been running your code through my cpu all night.",
    "Was that my CPU malfunctioning or did I just feel a spark between us?",

    #  Source: http://www.funcomet.com/for_kids/cool-robot-jokes/
    "Do robots have sisters?           No, just transistors!",

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

    r = e.recognize("(yes|no)", {})

    if not r or r.result == "no" or r.result == "":
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
