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
import functions

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class NormalStartStrategyCalcurator(StrategyCalcuratorBase):

    def reset_state(self):
        pass

    """
    referee_branchがNORMAL_STARTの場合のCalcurator。
    """
    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase

        pass_positions = [
            [2.0, 2.0],
            [6.0, 0.0],
        ]

        last_state = strategy_context.get_last("normal_strat_state")

        self.reset_state()
        is_shoot = False

        ball_pos = self._objects.ball.get_current_position()
        area = self._objects.robot[0].size_r + 0.2
        if functions.distance_btw_two_points(ball_pos, pass_positions[last_state]) < area:
            if len(pass_positions) > last_state + 1:
                cur_state = last_state + 1
            else:
                cur_state = last_state
        else:
            cur_state = last_state

        if len(pass_positions) == cur_state + 1:
            is_shoot = True

        # InitialStaticStrategyを元に組み立てる
        self._dynamic_strategy.clone_from(self._static_strategies['initial'])

        # 生きてるロボットにだけ新たな指示を出す例
        active_robot_ids = self._get_active_robot_ids()
        for idx, robot_id in enumerate(active_robot_ids):
            status = Status()
            if idx == 0:
                status.status = "keeper"
            elif idx == 1:
                status.status = "defence1"
            elif idx == 2:
                status.status = "defence2"
            elif idx == 3:
                status.status = "pass" if cur_state % 2 == 0 else "receive"
                if is_shoot and status.status == "receive":
                    status.status = "stop"
                status.pass_target_pos_x = pass_positions[cur_state][0]
                status.pass_target_pos_y = pass_positions[cur_state][1]
            elif idx == 4:
                status.status = "receive" if cur_state % 2 == 0 else "pass"
                if is_shoot and status.status == "receive":
                    status.status = "stop"
                status.pass_target_pos_x = pass_positions[cur_state][0]
                status.pass_target_pos_y = pass_positions[cur_state][1]

            self._dynamic_strategy.set_robot_status(robot_id, status)

        # self._dynamic_strategy.clone_from(self._static_strategies['initial'])
        result = self._dynamic_strategy

        strategy_context.update("normal_strat_state", cur_state)

        return result
