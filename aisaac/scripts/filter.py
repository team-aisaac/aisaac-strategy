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
    x, y, theta, x_sigma, y_sigma, theta_sigma = robot.get_current_position_for_filter()
    vx, vy, v_theta = robot.get_last_expected_velocity(theta=True)
    # 行動による信念アップデート
    x += vx * ROBOT_DT
    y += vy * ROBOT_DT
    theta += v_theta * ROBOT_DT
    theta %= 2 * np.pi # 2πでmod
    x_sigma += ACTION_POSITION_SIGMA
    y_sigma += ACTION_POSITION_SIGMA
    theta_sigma += ACTION_ORIENTATION_SIGMA

    # vision情報による信念アップデート
    vision_x, vision_y, vision_theta = robot.get_vision_position()
    vision_x_sigma = x_sigma * (1. / (x_sigma + VISION_POSITION_SIGMA))
    vision_y_sigma = y_sigma * (1. / (y_sigma + VISION_POSITION_SIGMA))
    vision_theta_sigma = theta_sigma * (1. / (theta_sigma + VISION_ORIENTATION_SIGMA))
    updated_x = x + vision_x_sigma * (vision_x - x)
    updated_y = y + vision_y_sigma * (vision_y - y)
    updated_theta = theta + vision_theta_sigma * (vision_theta - theta)
    updated_theta %= 2 * np.pi # 2πでmod
    updated_x_sigma = (1. - vision_x_sigma) * x_sigma
    updated_y_sigma = (1. - vision_y_sigma) * y_sigma
    updated_theta_sigma = (1. - vision_theta_sigma) * theta_sigma

    # アップデートした信念で上書き
    robot.set_current_position_for_filter(updated_x, updated_y, updated_theta, updated_x_sigma,updated_y_sigma, updated_theta_sigma)

def identity_filter(robot):
    x, y, theta = robot.get_vision_position()
    robot.set_current_position(x, y, theta)
