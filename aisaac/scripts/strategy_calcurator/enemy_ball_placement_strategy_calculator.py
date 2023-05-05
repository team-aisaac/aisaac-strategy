#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

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

class EnemyBallPlacement(StrategyCalcuratorBase):
    """
    referee_branchがFRIEND_BALL_PLACEMENTの場合のCalcurator。
    """
    def __init__(self, objects):
        super(EnemyBallPlacement, self).__init__(objects)
        self.kick_margin = 1.
        self.dribble_range = 1.5
        self.ball_dispersion = [10.] * 30
        self.leave_range = 0.5

    def calcurate(self, strategy_context=None, place_ball_position=[0,0]):
        # type: (StrategyContext, List) -> StrategyBase

        if self._detect_enemy_kick(strategy_context):
            strategy_context.update("enemy_kick", True, namespace="world_model")
            strategy_context.update("defence_or_attack", False, namespace="world_model")

        active_robot_ids = self._get_active_robot_ids()
        active_enemy_ids = self._get_active_enemy_ids()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        second_nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[1]
        for robot_id in active_robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "keeper"
            elif robot.get_role() == "LDF":
                status.status = "defence4"
            elif robot.get_role() == "RDF":
                status.status = "defence3"
            elif robot.get_role() == "LFW":
                #敵kickerとballの延長線上に移動
                status.status = "move_linear"
                # if second_nearest_enemy_id != None:
                if True:
                    # status.pid_goal_pos_x, status.pid_goal_pos_y = functions.calculate_internal_dividing_point(self._enemy[nearest_enemy_id].get_current_position()[0], self._enemy[nearest_enemy_id].get_current_position()[1], self._ball_params.get_current_position()[0], self._ball_params.get_current_position()[1], functions.distance_btw_two_points(self._enemy[nearest_enemy_id].get_current_position(), self._ball_params.get_current_position()) + 0.55, -0.55)
                    target_pos_xy = functions.calculate_internal_dividing_point_vector_args(
                        config.GOAL_CENTER, self._ball_params.get_current_position(), 1, 1)

                    status.pid_goal_pos_x = target_pos_xy[0]
                    status.pid_goal_pos_y = target_pos_xy[1]

                    status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]) )
            elif robot.get_role() == "RFW":
                
                status.status = "move_linear"
                
                # ボールプレースメントの場所とボールの場所でゴールに近い方のゴール側に陣取る

                goal_pos = np.array(config.GOAL_CENTER)
                place_pos = np.array(place_ball_position)
                ball_pos = np.array(self._ball_params.get_current_position())

                if np.linalg.norm(goal_pos - place_pos) < np.linalg.norm(goal_pos - ball_pos):
                    target_pos = place_pos + (goal_pos - place_pos) * 1 / np.linalg.norm(goal_pos - place_pos)
                else:
                    target_pos = ball_pos + (goal_pos - ball_pos) * 1 / np.linalg.norm(goal_pos - ball_pos)
                
                status.pid_goal_pos_x = target_pos[0]
                status.pid_goal_pos_y = target_pos[1]

                status.pid_goal_theta = np.arctan2(
                    (self._ball_params.get_current_position()[1] - robot.get_current_position()[1]),
                    (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]))
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)


        result = self._dynamic_strategy
        return result
