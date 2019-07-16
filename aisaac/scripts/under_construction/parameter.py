#!/usr/bin/env  python
# coding:utf-8
import itertools
import math
import rospy
import numpy as np
from consai_msgs.msg import Pose
from consai_msgs.msg import robot_commands
#from aisaac.srv import Kick
from aisaac.msg import Status
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import entity
import matplotlib.pyplot as plt
from matplotlib import animation
import functions
from robot_functions import RobotPid
ROBOT_LOOP_RATE = 60.
DT =  1. / ROBOT_LOOP_RATE
COEFFICIENT = 1.
MAX_SPEED =  3.7


class Parameter(object):
    def __init__(self, parameter_path):
        """
        parameterをガウス分布からの生成モデルとみなし、ベイズ更新していく。
        parameterはtextファイルから読み込んで利用する(?)
        無情報事前分布でもいいかも
        """
        self.mean = parameter_mean
        self.variance = parameter_variance

        # ここどうすっか
        self.const_variance = 1.0

    def calculate_posterior_distribution(self, data):
        self.mean, self.variance =  (self.variance * data + self.const_variance * self.mean) / (self.const_variance + self.variance), self.variance * self.const_variance /
        return self.mean
