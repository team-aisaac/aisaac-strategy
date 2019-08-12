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


class IndirectFreeBlue(StrategyCalcuratorBase):
    """
    referee_branchがINDIRECT_FREE_BLUEの場合のCalcurator。
    """
    def __init__(self, objects):
        self.friend = objects.robot
        self._robot_ids = objects.get_robot_ids()
        self._dynamic_strategy = DynamicStrategy()

    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase

        active_robot_ids = self._get_active_robot_ids()
        status = Status()
        for idx, robot_id in enumerate(active_robot_ids):
            if idx == 4:
                status.status = "pass"
                status.pass_target_pos_x = self.friend[5].get_current_position()[0]
                status.pass_target_pos_y = self.friend[5].get_current_position()[1]
            elif idx == 5:
                status.status = "receive"
                status.pass_target_pos_x = self.friend[5].get_current_position()[0]
                status.pass_target_pos_y = self.friend[5].get_current_position()[1]
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)
        result = self._dynamic_strategy

        return result
