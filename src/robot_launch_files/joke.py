#! /usr/bin/python

import sys
import random
from collections import deque

# ROS
import rospy
from geometry_msgs.msg import Twist
from smach_msgs.msg import SmachContainerStatus
from hmi import TimeoutException


# clue should be seperated with 15 spaces
jokes = [
    "What do you call a fish with no eyes?               A fsh.",
    "You don't need a parachute to go skydiving. You need a parachute to go skydiving twice.",
    "What is the difference between a snowman and a snowwoman?               Snowballs.",
    "What is Bruce Lee's favorite drink?               Wataaaaah!",
    "A blind man walks into a bar. And a table. And a chair.",
    "What is yellow and when you push the button, it turns red?               A chick in the blender",
    "Why was 6 afraid of 7?               Because 7, 8, 9",
    "What do you call a robot that always goes the long way around?               R 2 detour",
    "Can a kangaroo jump higher than a house?               Of course, a house does not jump at all.",
    "What should you put on the tomb stone of a mathematician?               He did not count with this...",
    "Why do cows wear bells?               Their horns do not work.",
    "What did one ocean say to the other?               Nothing, they just waved",
    "What does the roof say to the house?         I got you covered",
    "People used to laugh at me when I would say     I want to be a comedian    , well nobody's laughing now.",
    "My superpower is making people laugh. Which would be great if I was trying to be funny.",
    "Why did the physics teacher break up with the biology teacher?               There was no chemistry.",
    "Did you hear about the kidnapping at school?               It’s all right, he woke up."
]


class Joke:
    """
    Joke class
    """
    def __init__(self, robot, jokes=None, standby_min=20):
        """
        Constructor
        :param robot: Robot object
        :type robot: robot_skills.Robot
        :param jokes: Joke or list of jokes
        :type jokes: str or [str]
        :param standby_min: number of minutes of standby after which the robot will tell a joke
        :type standby_min: numerical
        """
        if isinstance(jokes, str):
            self._jokes = deque([jokes])
        elif isinstance(jokes, list) and isinstance(jokes[0], str):
            self._jokes = deque(jokes)
            random.shuffle(self._jokes)
        else:
            rospy.logerr("jokes should be a string or a list of strings")
            sys.exit(1)

        self._robot = robot
        self._standby_min = standby_min

        self._base_sub = rospy.Subscriber("base/references", Twist, self.callback)
        self._smach_sub = rospy.Subscriber("smach/container_status", SmachContainerStatus, self.callback)
        self._trigger = rospy.Timer(rospy.Duration(2), self.timer_callback)

        self._last_update = rospy.Time.now()

    def callback(self, data):
        """
        callback for robot being active
        :param data: unused msg data
        """
        self._last_update = rospy.Time.now()

    def timer_callback(self, event):
        """
        Timer callback to check if enough time has passed
        :param event: unused timer event
        """
        if (rospy.Time.now() - self._last_update).to_sec() > self._standby_min * 60:
            try:
                self.joke()
            except:
                self._last_update = rospy.Time.now()

    def joke(self):
        """
        Tell a joke from self._jokes
        """
        self._jokes.rotate()

        self._robot.speech.speak(self._jokes[0])

        rospy.sleep(4.0)

        self._robot.speech.speak("Would you like to hear another one?")

        r = None
        try:
            r = self._robot.hmi.query('', 'T -> yes | no', 'T').sentence
        except TimeoutException:
            pass

        if not r or r == "no":
            self._robot.speech.speak("Ok, I will be quiet for another %d minutes" % self._standby_min)
            self._last_update = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('joke')

    robot_name = rospy.get_namespace().split("/")[-2]
    if robot_name == "amigo":
        from robot_skills.amigo import Amigo
        robot = Amigo()
    elif robot_name == "sergio":
        from robot_skills.sergio import Sergio
        robot = Sergio()
    elif robot_name == "hero":
        from robot_skills.hero import Hero
        robot = Hero()
    else:
        rospy.loginfo("Unknown robot namespace %s" % robot_name)
        sys.exit(1)

    joke = Joke(robot, jokes, 20)

    rospy.spin()
