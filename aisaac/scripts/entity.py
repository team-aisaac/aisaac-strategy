#!/usr/bin/env  python
# coding:utf-8
import numpy as np
import random
from context import RobotContext
import config

class Entity(object):
    def __init__(self):
        self.field_x_size = 12000.
        self.field_y_size = 9000.
        self.size_r = 0.

        """
        position情報はvision_positionとcurrent_positionの2つを用意する
        _vision_position->_current_positionを2パターンで更新する。
        1. 自分以外のロボット情報およびボール情報
            vision以外の情報は利用できないので恒等関数
            _current_position = _vision_position
        2. 自分自身
            自分が移動する速度情報とvision情報両方を利用してアップデート
        """
        self._current_position_x = 0.
        self._current_position_y = 0.
        self._current_orientation = 0.

        self._vision_position_x = 0.
        self._vision_position_y = 0.
        self._vision_orientation = 0.

        self._max_velocity = 0.

    def set_current_position(self, x, y, theta=None):
        self._current_position_x = x
        self._current_position_y = y

        if theta is not None:
            self._current_orientation = theta

    def get_current_position(self, theta=False):
        if theta:
            return self._current_position_x, self._current_position_y, self._current_orientation
        else:
            return self._current_position_x, self._current_position_y

    def get_current_orientation(self):
        return self._current_orientation

    def set_current_velocity(self, vx, vy, vtheta=None):
        self._current_velocity_x = vx
        self._current_velocity_y = vy

        if vtheta is not None:
            self._current_velocity_orientation = vtheta

    def set_max_velocity(self, max_vel):
        self._max_velocity = max_vel

    def set_current_velocity_orientation(self, vtheta):
        self._current_velocity_orientation = vtheta

    def get_current_velocity(self):
        return self._current_velocity_x, self._current_velocity_y

    def get_current_velocity_orientation(self):
        return self._current_velocity_orientation

    def get_max_velocity(self):
        return self._max_velocity

    def set_future_position(self, x, y, theta=None):
        self._future_position_x = x
        self._future_position_y = y

        if theta is not None:
            self._future_orientation = theta

    def set_future_orientation(self, theta):
        self._future_orientation = theta

    def get_future_position(self, theta=False):
        if theta:
            return self._future_position_x, self._future_position_y, self._future_orientation
        else:
            return self._future_position_x, self._future_position_y

    def get_future_orientation(self):
        return self._future_orientation



class Robot(Entity):
    def __init__(self, id=None):
        super(Robot, self).__init__()
        if id is not None:
            self._id = id

        else:
            self._id = None

        self.size_r = 90.0 / 1000

        self.front_degree = 11.86 * 2
        self.max_velocity = 2000. #mm/s

        self._has_a_ball = False

        self._pass_target_pos_x = 0.
        self._pass_target_pos_y = 0.

        self._robot_context = None  # type: context.RobotContext

        self._role = "" # GK などのポジション
        self.velocity_surge = 0
        self.velocity_sway = 0
        self.omega = 0

        self._current_position_x_sigma = config.INITIAL_POSITION_SIGMA
        self._current_position_y_sigma = config.INITIAL_POSITION_SIGMA
        self._current_orientation_sigma = config.INITIAL_POSITION_SIGMA

        self._robot_context = RobotContext()
        self._robot_context.register_new_context("vel_xyr", 2, (0.0, 0.0, 0.0))

    def get_id(self):
        return self._id

    def get_role(self):
        return self._role

    def set_role(self, role):
        self._role = role

    def has_a_ball(self, ball_x, ball_y):
        if (self.get_current_position()[0] - ball_x) ** 2 + (self.get_current_position()[1] - ball_y) ** 2 < (self.size_r + 0.003) **2:
            return True
        else:
            return False

    def set_pass_target_position(self, x, y):
        self._pass_target_pos_x = x
        self._pass_target_pos_y = y

    # カルマンフィルタ用のセッター
    def set_current_position_for_filter(self, x, y, theta, x_sigma, y_sigma, theta_sigma):
        self._current_position_x = x
        self._current_position_y = y
        self._current_orientation = theta
        self._current_position_x_sigma = x_sigma
        self._current_position_y_sigma = y_sigma
        self._current_orientation_sigma = theta_sigma

    # カルマンフィルタ用のゲッター
    def get_current_position_for_filter(self):
        return self._current_position_x, self._current_position_y, self._current_orientation, \
                self._current_position_x_sigma, self._current_position_y_sigma, self._current_orientation_sigma

    def get_vision_position(self):
        return self._vision_position_x, self._vision_position_y, self._vision_orientation

    def set_vision_position(self, x, y, theta):
        self._vision_position_x = x
        self._vision_position_y = y
        self._vision_orientation = theta

    # ロボットの持つ信念を初期化する
    def reset_own_belief(self):
        self._current_position_x_sigma = config.INITIAL_POSITION_SIGMA
        self._current_position_y_sigma = config.INITIAL_POSITION_SIGMA
        self._current_orientation_sigma = config.INITIAL_POSITION_SIGMA

    # def set_current_position(self, x, y, theta):
    #     None
    # def set_current_velosity(self, x, y, theta):
    #     None

    # def get_future_position(self):
    #     return self.future_position_x, self.future_position_y, self.future_orientation

    def get_pass_target_position(self):
        return self._pass_target_pos_x, self._pass_target_pos_y

    def get_last_expected_velocity(self, theta=False):
        last_vel_xyr = self._robot_context.get_last("vel_xyr")  # (Vx, Vy, Vr)
        if theta:
            return last_vel_xyr
        else:
            return last_vel_xyr[0], last_vel_xyr[1]

    def update_expected_velocity_context(self, Vx, Vy, Vr):
        if self._robot_context:
            self._robot_context.update("vel_xyr", (Vx, Vy, Vr))

    def handle_loop_callback(self):
        self._robot_context.handle_loop_callback()


class Ball(Entity):
    def __init__(self):
        super(Ball, self).__init__()
        self.size_r = 21.5 / 1000

        self._line_a = 0
        self._line_b = 0

    def get_line_a(self):
        return self._line_a

    def get_line_b(self):
        return self._line_b

    def set_line_a(self, a):
        self._line_a = a

    def set_line_b(self, b):
        self._line_b = b
