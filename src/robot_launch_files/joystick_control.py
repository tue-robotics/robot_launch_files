#! /usr/bin/python

import sys

# ROS
import rospy
from sensor_msgs.msg import Joy


class ControllerJoystick:
    """
    Joke class
    """
    def __init__(self, robot):
        """
        Constructor
        :param robot: Robot object
        :type robot: robot_skills.Robot
        """

        self._robot = robot
        self._data = Joy()
        self._base_sub = rospy.Subscriber("joy", Joy, self.callback)

    def callback(self, data):
        """
        callback for robot being active
        :param data: Joy msg data
        """
        self._data = data

    def process_controller_data(self):
        """
        This function should be called at a sufficiently high frequency
        :return:
        """
        for button, i in enumerate(self._data.buttons):
            if button:
                rospy.loginfo("button {} pressed".format(i))
                self._test_function()
        if self._data.buttons[6] or self._data.buttons[7] or self._data.buttons[8]:
            x_move = self._data.buttons[6]
            y_move = self._data.buttons[7]
            th_move = self._data.buttons[8]
            self._move_base(x_move, y_move, th_move)

    def _move_base(self, x, y, th):
        self._robot.base.force_drive(x, y, th, 0.1)

    def _test_function(self):
        rospy.loginfo("In this case, move base")


if __name__ == '__main__':
    rospy.init_node('joystick_control')

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

    controller = ControllerJoystick(robot)
    rate = rospy.Rate(10)  # ROS Rate at 5Hz

    while not rospy.is_shutdown():
        controller.process_controller_data()
        rospy.loginfo("Process data")
        rate.sleep()

    rospy.spin()
