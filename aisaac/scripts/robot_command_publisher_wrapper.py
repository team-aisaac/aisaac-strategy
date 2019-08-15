#!/usr/bin/env python
# coding:utf-8
from consai_msgs.msg import robot_commands
import rospy
import math

class RobotCommandPublisherWrapper(object):
    def __init__(self, robot_color, robot_id):
        self._publisher = rospy.Publisher(
            "/" + robot_color + "/robot_" + robot_id + "/robot_commands",
            robot_commands,
            queue_size=10)

    def publish(self, msg):
        # type: (robot_commands) -> None

        kick_power_threshold = [0.1, 1.0, 2.0, 3.0]
        sim_kick_speed = math.sqrt(msg.kick_speed_x**2 + msg.kick_speed_z**2)

        if sim_kick_speed < kick_power_threshold[0]:
            real_kick_power = 0
        elif kick_power_threshold[0] <= sim_kick_speed < kick_power_threshold[1]:
            real_kick_power = 1
        elif kick_power_threshold[1] <= sim_kick_speed < kick_power_threshold[2]:
            real_kick_power = 2
        elif kick_power_threshold[2] <= sim_kick_speed < kick_power_threshold[3]:
            real_kick_power = 3
        elif kick_power_threshold[3] <= sim_kick_speed:
            real_kick_power = 4

        msg.kick_on_flag = True
        msg.kick_power = real_kick_power

        if msg.kick_speed_z > 0.0:
            msg.chip_on_flag = True
        else:
            msg.chip_on_flag = False

        self._publisher.publish(msg)

    def handle_loop_event(self):
        self._publisher.publish
