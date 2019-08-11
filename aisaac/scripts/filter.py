#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import config

ROBOT_LOOP_RATE = config.ROBOT_LOOP_RATE
ROBOT_DT = 1. / ROBOT_LOOP_RATE

VISION_POSITION_SIGMA = config.VISION_POSITION_SIGMA
VISION_ORIENTATION_SIGMA = config.VISION_ORIENTATION_SIGMA
ACTION_POSITION_SIGMA = config.ACTION_POSITION_SIGMA
ACTION_ORIENTATION_SIGMA = config.ACTION_ORIENTATION_SIGMA

def kalman_filter(robot):
    vision_x, vision_y, vision_theta = robot.get_vision_position()

    "0.00のときは通信がはじまっていないので何もしない"
    if vision_x == 0.00 and vision_y == 0.00 and vision_theta == 0.00:
        return None

    x, y, theta, x_sigma, y_sigma, theta_sigma = robot.get_current_position_for_filter()
    vx, vy, v_theta = robot.get_last_expected_velocity(theta=True)
    # 行動による信念アップデート
    x += vx * ROBOT_DT
    y += vy * ROBOT_DT
    theta += v_theta * ROBOT_DT
    theta += np.pi
    theta %= 2 * np.pi # 2πでmod
    theta -=  np.pi
    x_sigma += ACTION_POSITION_SIGMA
    y_sigma += ACTION_POSITION_SIGMA
    theta_sigma += ACTION_ORIENTATION_SIGMA

    # vision情報による信念アップデート
    vision_x, vision_y, vision_theta = robot.get_vision_position()

    x_gain = x_sigma / (x_sigma + VISION_POSITION_SIGMA)
    y_gain = y_sigma / (y_sigma + VISION_POSITION_SIGMA)
    theta_gain = theta_sigma / (theta_sigma + VISION_ORIENTATION_SIGMA)
    updated_x = x + x_gain * (vision_x - x)
    updated_y = y + y_gain * (vision_y - y)

    d_theta = vision_theta - theta
    if d_theta < 0 and abs(d_theta) > np.pi:
        d_theta = vision_theta - (theta - 2 * np.pi)

    if d_theta > 0 and abs(d_theta) > np.pi:
        d_theta = (vision_theta - 2 * np.pi) - theta

    updated_theta = theta + theta_gain * d_theta

    updated_theta +=  np.pi
    updated_theta %= 2 * np.pi # 2πでmod
    updated_theta -= np.pi

    updated_x_sigma = (1. - x_gain) * x_sigma
    updated_y_sigma = (1. - y_gain) * y_sigma
    updated_theta_sigma = (1. - theta_gain) * theta_sigma

    # if  abs(vision_theta - updated_theta) > 1:
    #     print robot.get_id(), "thetaがずれてる", vision_theta - updated_theta
    #
    # if  abs(vision_x- updated_x)>0.05:
    #     print robot.get_id(), "xがずれてる", vision_x - updated_x
    #
    # if  abs(vision_y - updated_y)>0.05:
    #     print robot.get_id(), "yがずれてる", vision_y - updated_y

    # アップデートした信念で上書き
    robot.set_current_position_for_filter(updated_x, updated_y, updated_theta, updated_x_sigma,updated_y_sigma, updated_theta_sigma)

def identity_filter(robot):
    x, y, theta = robot.get_vision_position()
    robot.set_current_position(x, y, theta)
