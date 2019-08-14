#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from abc import ABCMeta, abstractmethod
import functions

from strategy import StrategyBase, InitialStaticStrategy, StopStaticStrategy, DynamicStrategy
from strategy_calcurator import StrategyCalcuratorBase
from context import StrategyContext
from objects import Objects
from aisaac.msg import Status
import copy

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class IndirectFreeDefence(StrategyCalcuratorBase):
    """
    referee_branchがINDIRECT_FREE_DEFENCEの場合のCalcurator。
    """
    def __init__(self, objects):
        self._robot = objects.robot
        self._enemy = objects.enemy
        self._robot_ids = objects.get_robot_ids()
        self._enemy_ids = objects.get_enemy_ids()
        self._ball_params = objects.ball
        self._dynamic_strategy = DynamicStrategy()
        self._objects = objects

    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase
        active_robot_ids = self._get_active_robot_ids()
        active_enemy_ids = self._get_active_enemy_ids()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        for robot_id in active_robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "keeper"
            elif robot.get_role() == "LDF":
                status.status = "defence1"
            elif robot.get_role() == "RDF":
                status.status = "defence2"
            elif robot.get_role() == "LFW":
                #敵kickerとballの延長線上に移動
                status.status = "move_linear"
                if nearest_enemy_id != None:
                    status.pid_goal_pos_x, status.pid_goal_pos_y = functions.calculate_internal_dividing_point(self._enemy[nearest_enemy_id].get_current_position()[0], self._enemy[nearest_enemy_id].get_current_position()[1], self._ball_params.get_current_position()[0], self._ball_params.get_current_position()[1], functions.distance_btw_two_points(self._enemy[nearest_enemy_id].get_current_position(), self._ball_params.get_current_position()) + 0.55, -0.55)
                    status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - self._robot[3].get_current_position()[1]) , (self._ball_params.get_current_position()[0] - self._robot[2].get_current_position()[0]) )
            elif robot.get_role() == "RFW":
                #フリーで最もゴールに近い敵idを返す
                status.status = "move_linear"
                free_enemy_id = self._get_free_enemy_id(4, nearest_enemy_id)
                status.pid_goal_pos_x = (self._ball_params.get_current_position()[0] + self._enemy[free_enemy_id].get_current_position()[0]) / 2
                status.pid_goal_pos_y = (self._ball_params.get_current_position()[1] + self._enemy[free_enemy_id].get_current_position()[1]) / 2
                status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - self._robot[4].get_current_position()[1]) , (self._ball_params.get_current_position()[0] - self._robot[4].get_current_position()[0]) )
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)


        result = self._dynamic_strategy
        return result
