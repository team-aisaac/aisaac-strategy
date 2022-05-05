#!/usr/bin/env  python
# coding:utf-8
import math

ROBOT_LOOP_RATE = 60.
WORLD_LOOP_RATE = 60.

NUM_FRIEND_ROBOT = 5
NUM_ENEMY_ROBOT = 16

ROBOT_MAX_VELOCITY = 0.1  # m/s 機体の最高速度
ROBOT_DRIBBLE_VELOCITY = 0.05 #ドリブル速度

MAX_KICK_POWER_X = math.sqrt(12.0) * 2.0
MAX_KICK_POWER_Z = math.sqrt(12.0) * 2.0

# カルマンフィルタ用
INITIAL_POSITION_SIGMA = 1
INITIAL_ORIENTATION_SIGMA = 1.
VISION_POSITION_SIGMA = 0.01 ** 2
VISION_ORIENTATION_SIGMA = 0.02 ** 2
ACTION_POSITION_SIGMA = 0.1 ** 2
ACTION_ORIENTATION_SIGMA = 0.1 ** 2

HAS_A_BALL_DISTANCE_THRESHOLD = 0.03

# フィールドサイズ
FIELD_SIZE = [13.4, 10.4]
# コートサイズ
COURT_SIZE = [12., 9]
# 守る(味方)側からみてLRしている
GOAL_CENTER = [-6.0, 0.0]
GOAL_LEFT = [-6.0, 0.62]
GOAL_RIGHT = [-6.0, -0.62]

# 攻める(味方)側からみてLRしている
GOAL_ENEMY_CENTER = [6.0, 0.0]
GOAL_ENEMY_LEFT = [6.0, 0.62]
GOAL_ENEMY_RIGHT = [6.0, -0.62]

PENALTY_AREA_FRIEND_LEFT = -6.0
PENALTY_AREA_FRIEND_RIGHT = -4.8
PENALTY_AREA_FRIEND_TOP = 1.2
PENALTY_AREA_FRIEND_BOTTOM = -1.2

PENALTY_AREA_ENEMY_LEFT = 4.8
PENALTY_AREA_ENEMY_RIGHT = 6.0
PENALTY_AREA_ENEMY_TOP = 1.2
PENALTY_AREA_ENEMY_BOTTOM = -1.2
