#!/usr/bin/env  python
# coding:utf-8

ROBOT_LOOP_RATE = 60.
WORLD_LOOP_RATE = 60.

NUM_FRIEND_ROBOT = 5
NUM_ENEMY_ROBOT = 8

ROBOT_MAX_VELOCITY = 3.0  # m/s 機体の最高速度

# カルマンフィルタ用
INITIAL_POSITION_SIGMA = 1
INITIAL_ORIENTATION_SIGMA = 1.
VISION_POSITION_SIGMA = 0.01 ** 2
VISION_ORIENTATION_SIGMA = 0.02 ** 2
ACTION_POSITION_SIGMA = 0.1 ** 2
ACTION_ORIENTATION_SIGMA = 0.1 ** 2

HAS_A_BALL_DISTANCE_THRESHOLD = 0.05
