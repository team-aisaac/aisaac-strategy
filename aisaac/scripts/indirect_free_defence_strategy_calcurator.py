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
        self.friend = objects.robot
        self.enemy = objects.enemy
        self._robot_ids = objects.get_robot_ids()
        self._enemy_ids = objects.get_enemy_ids()
        self.ball_params = objects.ball
        self._dynamic_strategy = DynamicStrategy()
        self._objects = objects

    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase

        active_robot_ids = self._get_active_robot_ids()
        active_enemy_ids = self._get_active_enemy_ids()
        status = Status()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        for idx, robot_id in enumerate(active_robot_ids):
            if idx == 0:
                status.status = "keeper"
            elif idx == 1:
                status.status = "defence1"
            elif idx == 2:
                status.status = "defence2"
            elif idx == 3:
                status.status = "move_linear"
                print nearest_enemy_id
                if nearest_enemy_id != None:
                    status.pid_goal_pos_x, status.pid_goal_pos_y = functions.calculate_internal_dividing_point(self.enemy[nearest_enemy_id].get_current_position()[0], self.enemy[nearest_enemy_id].get_current_position()[1], self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], functions.distance_btw_two_points(self.enemy[nearest_enemy_id].get_current_position(), self.ball_params.get_current_position()) + 0.55, -0.55)
                    status.pid_goal_theta = math.atan2( (self.ball_params.get_current_position()[1] - self.friend[3].get_current_position()[1]) , (self.ball_params.get_current_position()[0] - self.friend[3].get_current_position()[0]) )
            #パスカット実装予定
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)


        result = self._dynamic_strategy
        return result
