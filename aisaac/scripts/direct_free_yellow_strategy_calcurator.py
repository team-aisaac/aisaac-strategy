#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
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


class DirectFreeBlue(StrategyCalcuratorBase):

    referee_branchがDIRECT_FREE_yellowの場合のCalcurator。
    
    def __init__(self, objects):
        self.friend = objects.robot
        self._robot_ids = objects.get_robot_ids()
        self.ball_params = objects.ball
        self._dynamic_strategy = DynamicStrategy()

    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase

        active_robot_ids = self._get_active_robot_ids()
        status = Status()
        for idx, robot_id in enumerate(active_robot_ids):
            if idx == 0:
                status.status = "keeper"
            elif idx == 1:
                status.status = "defence1"
            elif idx == 2:
                status.status = "defence2"
            elif idx == 3:
                status.status = "move_linear"
                status.pid_goal_pos_x = (functions.calculate_internal_dividing_point(goal_x, goal_y, self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], functions.distance_btw_two_points(goal_pos, self.ball_params)))
                status.pid_goal_pos_y =
                status.pid_goal_theta =
            self._dynamic_strategy.set_robot_status(robot_id, status)
        result = self._dynamic_strategy

        return result
"""
