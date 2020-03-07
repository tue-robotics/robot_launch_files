#! /usr/bin/python

import sys

# ROS
import rospy
from sensor_msgs.msg import Joy
from robot_skills import arms


class ControllerJoystick:
    """
    ControllerJoystick class
    """
    def __init__(self, robot):
        """
        Constructor
        :param robot: Robot object
        :type robot: robot_skills.Robot
        """

        self._robot = robot
        self._arm = self._robot.get_arm(required_gripper_types=[arms.GripperTypes.GRASPING])
        self._data = Joy()
        self._base_sub = rospy.Subscriber("/joy", Joy, self.callback)

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
        if len(self._data.buttons) == 0:
            rospy.loginfo("Buttons list was empty")
            return

        if self._data.buttons[0]:
            self._close_gripper()

        if self._data.axes[0] or self._data.axes[1] or self._data.axes[3]:
            x_move = self._data.axes[1]
            y_move = self._data.axes[0]
            th_move = self._data.axes[3]
            self._move_base(x_move/5, y_move/5, th_move)
        return

    def _move_base(self, x, y, th):
        rospy.loginfo("moving base with x:{}, y:{}, th{}".format(x, y, th))
        self._robot.base.force_drive(x, y, th, 0.5)

    def _close_gripper(self):
        rospy.loginfo("closing gripper")
        self._arm.send_gripper_goal("close")


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
    rate = rospy.Rate(10.0)  # ROS Rate at 10Hz

    while not rospy.is_shutdown():
        controller.process_controller_data()
        rate.sleep()

    rospy.logdebug("While loop exited")
