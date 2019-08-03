#!/usr/bin/env  python
# coding:utf-8
import math
import numpy as np
import entity
import sys
import time
import concurrent.futures
from sklearn.cluster import KMeans
import rospy
#from consai_msgs.msg import VisionPacket, VisionIDList
from consai_msgs.msg import robot_commands
from consai_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
from consai_msgs.msg import RefereeTeamInfo
from std_msgs.msg import Int8
import struct
import serial
import time
#from aisaac.srv import Kick
from aisaac.msg import Status

from world_state import WorldState
from referee import Referee
import config

WORLD_LOOP_RATE = config.WORLD_LOOP_RATE

"""---点から直線に垂線を引いた時の接点(x, y)の計算---"""
def calculate_contact(a, b, c, x_0, y_0):
    denominator = 1 / (a**2 + b**2)
    x = denominator * (b**2 * x_0 - a * b * y_0 - a * c)
    y = denominator * (-a * b * x_0 + a**2 * y_0 - b * c)
    return x, y

"""---点と直線の距離の計算---"""
def distance_of_a_point_and_a_straight_line(x_0, y_0, a, b, c):
    d = abs(a * x_0 + b * y_0 + c) / np.sqrt(a**2 + b**2)
    return d

"""---2点をつなぐ直線ax+by+cのa,b,cを返す---"""
def line_parameters(x_1, y_1, x_2, y_2):
    a = y_1 - y_2
    b = x_2 - x_1
    c = x_1 * y_2 - x_2 * y_1
    return a, b, c

"""---速度から遠心力を計算---"""
def calculate_centrifugalforce(vel_x_before, vel_y_before, vel_x_after, vel_y_after):
    numerator = np.sqrt( vel_x_after ** 2 + vel_y_after ** 2 ) ** 3
    denominator = vel_x_after * (vel_y_after - vel_y_before) - vel_y_after * (vel_x_after - vel_x_before)
    # 遠心力の絶対値を計算
    R = numerator / denominator
    # 進行方向から遠心力方向の単位ベクトルを計算し、遠心力をベクトルで返す
    unit_vec_x = vel_x_after / np.sqrt(vel_x_after**2 + vel_y_after**2)
    unit_vec_y = -1 * vel_y_after / np.sqrt(vel_x_after**2 + vel_y_after**2)
    return R*unit_vec_x, R*unit_vec_y
