#!/usr/bin/env  python
# coding:utf-8

import math
import numpy as np
import config

WORLD_LOOP_RATE = config.WORLD_LOOP_RATE

"""---点から直線に垂線を引いた時の接点(x, y)の計算---"""
def calculate_contact(a, b, c, x_0, y_0):
    denominator = 1 / (a**2 + b**2)
    x = denominator * (b**2 * x_0 - a * b * y_0 - a * c)
    y = denominator * (-a * b * x_0 + a**2 * y_0 - b * c)
    return x, y


"""---二点間距離計算---"""
def distance_btw_two_points(point_a, point_b):
    return math.sqrt((point_a[0] - point_b[0])**2
                     + (point_a[1] - point_b[1])**2)

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

def line_parameters_vector_args(point_1, point_2):
    x_1 = point_1[0]
    y_1 = point_1[1]
    x_2 = point_2[0]
    y_2 = point_2[1]
    return line_parameters(x_1, y_1, x_2, y_2)

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

"""---m:nの内分点計算---"""
def calculate_internal_dividing_point(x_0, y_0, x_1, y_1, m, n):
    if m + n == 0:
        return 0, 0
    else:
        x = ((n * x_0) + (m * x_1)) / (m + n)
        y = ((n * y_0) + (m * y_1)) / (m + n)
        return x, y

def calculate_internal_dividing_point_vector_args(point_0, point_1, m, n):
    x_0 = point_0[0]
    y_0 = point_0[1]
    x_1 = point_1[0]
    y_1 = point_1[1]
    return calculate_internal_dividing_point(x_0, y_0, x_1, y_1, m, n)

def in_penalty_area(point_xy):
    x = point_xy[0]
    y = point_xy[1]

    penalty_area_range_l = [
        [-6.0, -4.8],
        [-1.2, 1.2]
    ]
    penalty_area_range_r = [
        [4.8, 6.0],
        [-1.2, 1.2]
    ]

    if penalty_area_range_l[0][0] < x < penalty_area_range_l[0][1] \
            and penalty_area_range_l[1][0] < y < penalty_area_range_l[1][1]:
        return "friend"
    if penalty_area_range_r[0][0] < x < penalty_area_range_r[0][1] \
            and penalty_area_range_r[1][0] < y < penalty_area_range_r[1][1]:
        return "enemy"
    return False

def cross_point(line_1, line_2):
    """
    Parameters
    ----------
    line_1: (a, b, c): ax + by + c = 0のa, b, cのリスト
    line_2: (a, b, c): ax + by + c = 0のa, b, cのリスト
    """
    l1a = line_1[0]
    l1b = line_1[1]
    l1c = line_1[2]

    l2a = line_2[0]
    l2b = line_2[1]
    l2c = line_2[2]

    x = (l1b * l2c - l2b * l1c) / (l1a * l2b - l2a * l1b)
    y = (l2a * l1c - l1a * l2c) / (l1a * l2b - l2a * l1b)

    return x, y
