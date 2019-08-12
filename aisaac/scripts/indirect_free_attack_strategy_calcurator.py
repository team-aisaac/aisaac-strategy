#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from abc import ABCMeta, abstractmethod

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


class IndirectFreeAttack(StrategyCalcuratorBase):
    """
    referee_branchがINDIRECT_FREE_BLUEで自分もBLUE、または
    referee_branchがINDIRECT_FREE_YELLOWで自分もYELLOW の場合のCalcurator。
    """
    def __init__(self, objects):
        self.objects = objects
        self.friend = objects.robot
        self._robot_ids = objects.get_robot_ids()
        self._dynamic_strategy = DynamicStrategy()

    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase
        sorted_ids = object.get_robot_ids_sorted_by_distance([6.0,0.0])
        print sorted_ids
            
        active_robot_ids = self._get_active_robot_ids()
        status = Status()
        for idx, robot_id in enumerate(active_robot_ids):
            if idx == 3:
                status.status = "pass"
                status.pass_target_pos_x = self.friend[4].get_current_position()[0]
                status.pass_target_pos_y = self.friend[4].get_current_position()[1]
            elif idx == 4:
                status.status = "receive"
                status.pass_target_pos_x = self.friend[4].get_current_position()[0]
                status.pass_target_pos_y = self.friend[4].get_current_position()[1]
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)
        result = self._dynamic_strategy

        return result
