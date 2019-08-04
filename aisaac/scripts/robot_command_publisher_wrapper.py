#!/usr/bin/env python
# coding:utf-8
from consai_msgs.msg import robot_commands
import rospy


class RobotCommandPublisherWrapper(object):
    def __init__(self, robot_color, robot_id):
        self._publisher = rospy.Publisher(
            "/" + robot_color + "/robot_" + robot_id + "/robot_commands",
            robot_commands,
            queue_size=10)

    def publish(self, msg):
        self._publisher.publish(msg)

    def handle_loop_event(self):
        self._publisher.publish
