#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#

import rospy

class Logger(object):
    def __init__(self, binded_obj):
        self.binded_obj = binded_obj

    def loginfo(self, string):
        rospy.loginfo(
            "\033[34m" +
            self.binded_obj.__class__.__name__ +
            ":\033[0m " + str(string))

    def logerr(self, string):
        rospy.logerr(
            "\033[34m" +
            self.binded_obj.__class__.__name__ +
            ":\033[0m " + str(string))

    def logdebug(self, string):
        rospy.logdebug(
            "\033[34m" +
            self.binded_obj.__class__.__name__ +
            ":\033[0m " + str(string))

    def loginfo_throttle(self, sec, string):
        rospy.loginfo_throttle(sec,
            "\033[34m" +
            self.binded_obj.__class__.__name__ +
            ":\033[0m " + str(string))

    def logerr_throttle(self, sec, string):
        rospy.logerr_throttle(sec,
            "\033[34m" +
            self.binded_obj.__class__.__name__ +
            ":\033[0m " + str(string))

    def logdebug_throttle(self, sec, string):
        rospy.logdebug_throttle(sec,
            "\033[34m" +
            self.binded_obj.__class__.__name__ +
            ":\033[0m " + str(string))
