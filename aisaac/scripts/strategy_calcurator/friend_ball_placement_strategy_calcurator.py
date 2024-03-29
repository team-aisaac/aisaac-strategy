#!/usr/bin/env python
# -*- coding: utf-8 -*-

from .strategy import StrategyBase
from .strategy_calcurator import StrategyCalcuratorBase
from common.context import StrategyContext
from aisaac.msg import Status
from common import functions
import config
import numpy as np

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")

class FriendBallPlacement(StrategyCalcuratorBase):
    """
    referee_branchがFRIEND_BALL_PLACEMENTの場合のCalcurator。
    """
    def __init__(self, objects):
        super(FriendBallPlacement, self).__init__(objects)
        self.kick_margin = 1.
        self.dribble_range = 1.5
        self.ball_dispersion = [10.] * 30
        self.leave_range = 0.5

    def calcurate(self, strategy_context=None, place_ball_position=[0,0]):
        # type: (StrategyContext, List) -> StrategyBase
        self.place_point = place_ball_position
        self.ball_dispersion.append(
            functions.distance_btw_two_points(self.place_point, self._ball_params.get_current_position()))
        del self.ball_dispersion[0]
        ball_dispersion_average = sum(self.ball_dispersion)/len(self.ball_dispersion)

        #place point,ボールから一番近い機体設定
        self.active_robot_ids = self._get_active_robot_ids()
        robots_near_to_ball_ids = self._objects.get_robot_ids_sorted_by_distance_to_ball(self.active_robot_ids)
        robots_near_to_placement_point_ids = self._objects.get_robot_ids_sorted_by_distance(self.place_point, self.active_robot_ids)
        receive_robot_id = robots_near_to_placement_point_ids[0]
        if robots_near_to_placement_point_ids[0] == robots_near_to_ball_ids[0]:
            kick_robot_id = robots_near_to_ball_ids[1]
        else:
            kick_robot_id = robots_near_to_ball_ids[0]

        #ボールがplace pointから0.15m以内ならボールから離れる
        if ball_dispersion_average < 0.15:
            result = self.leave_ball()
        #ボール後方に回り込めるならball placement
        elif self.judge_kick_space(kick_robot_id):
            result = self.ball_placement(kick_robot_id, receive_robot_id)
        #回り込めない場合ボールを移動
        else:
            result = self.change_ball_position(kick_robot_id)

        return result

    #ball後ろに回り込めるか判定
    def judge_kick_space(self, kick_robot_id):
        n = self.kick_margin
        m = functions.distance_btw_two_points(self.place_point, self._ball_params.get_current_position()) + n
        moving_point = functions.calculate_internal_dividing_point_vector_args(self.place_point, self._ball_params.get_current_position(), m, n)

        kick_robot = self._objects.get_robot_by_id(kick_robot_id)

        #ボール後方がフィールド内かで判定
        offset = 1.5
        if -config.FIELD_SIZE[0] / 2. + (kick_robot.size_r * offset)  < moving_point[0] < config.FIELD_SIZE[0] / 2. - (kick_robot.size_r * offset) \
                and -config.FIELD_SIZE[1] / 2. + (kick_robot.size_r * offset)  < moving_point[1] < config.FIELD_SIZE[1] / 2. - (kick_robot.size_r * offset):
            return True
        else:
            return False

    def ball_placement(self, kick_robot_id, receive_robot_id):
        #place pointからボールが遠ければパス,近ければドリブル
        if functions.distance_btw_two_points(self.place_point, self._ball_params.get_current_position()) > self.dribble_range:
            result = self.ball_placement_kick(kick_robot_id, receive_robot_id)
        else:
            result = self.ball_placement_dribble(kick_robot_id, receive_robot_id)

        return result

    def ball_placement_kick(self, kick_robot_id, receive_robot_id):
        #placementに関係ない機体はstop
        #今後変更予定
        status = Status()
        for robot_id in self.active_robot_ids:
            status.status = "stop"
            self._dynamic_strategy.set_robot_status(robot_id, status)

        kick_robot_status = Status()
        kick_robot_status.status = "ball_place_kick"
        kick_robot_status.pass_target_pos_x = self.place_point[0]
        kick_robot_status.pass_target_pos_y = self.place_point[1]
        self._dynamic_strategy.set_robot_status(kick_robot_id, kick_robot_status)

        receive_robot_status = Status()
        receive_robot_status.status = "receive"
        receive_robot = self._objects.get_robot_by_id(receive_robot_id)
        #機体半径分レシーブ位置を下げる
        vector = np.array(self._ball_params.get_current_position()) - np.array(self.place_point)
        vector = (receive_robot.size_r / np.linalg.norm(vector)) * vector
        vector = np.array(self.place_point) - vector

        receive_robot_status.pid_goal_pos_x = vector[0]
        receive_robot_status.pid_goal_pos_y = vector[1]
        self._dynamic_strategy.set_robot_status(receive_robot_id, receive_robot_status)

        result = self._dynamic_strategy
        return result

    def ball_placement_dribble(self, kick_robot_id, receive_robot_id):
        #placementに関係ない機体はstop
        #今後変更予定
        status = Status()
        for robot_id in self.active_robot_ids:
            status.status = "stop"
            self._dynamic_strategy.set_robot_status(robot_id, status)

        kick_robot_status = Status()
        kick_robot_status.status = "ball_place_dribble"
        kick_robot_status.pass_target_pos_x = self.place_point[0]
        kick_robot_status.pass_target_pos_y = self.place_point[1]
        self._dynamic_strategy.set_robot_status(kick_robot_id, kick_robot_status)

        receive_robot_status = Status()
        receive_robot_status.status = "receive"
        receive_robot = self._objects.get_robot_by_id(receive_robot_id)
        #機体半径分レシーブ位置を下げる
        vector = np.array(self._ball_params.get_current_position()) - np.array(self.place_point)
        vector = (receive_robot.size_r / np.linalg.norm(vector)) * vector
        vector = np.array(self.place_point) - vector

        receive_robot_status.pid_goal_pos_x = vector[0]
        receive_robot_status.pid_goal_pos_y = vector[1]
        self._dynamic_strategy.set_robot_status(receive_robot_id, receive_robot_status)

        result = self._dynamic_strategy
        return result

    def leave_ball(self):
        status = Status()
        for robot_id in self.active_robot_ids:
            robot = self._objects.get_robot_by_id(robot_id)
            distance_ball_robot = functions.distance_btw_two_points(robot.get_current_position(), self._ball_params.get_current_position())
            if distance_ball_robot < self.leave_range:
                vector = np.array(self._ball_params.get_current_position()) - np.array(robot.get_current_position())
                vector = (self.leave_range / np.linalg.norm(vector)) * vector
                vector = np.array(robot.get_current_position()) - vector
                status.pid_goal_pos_x = vector[0]
                status.pid_goal_pos_y = vector[1]
                status.status = "move_linear"
            else:
                status.status = "stop"

            self._dynamic_strategy.set_robot_status(robot_id, status)
        result = self._dynamic_strategy
        return result
