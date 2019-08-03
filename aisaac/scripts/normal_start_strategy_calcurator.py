#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from abc import ABCMeta, abstractmethod

from strategy import StrategyBase, InitialStaticStrategy, StopStaticStrategy, DynamicStrategy
from strategy_calcurator import StrategyCalcuratorBase
from strategy_context import StrategyContext
from objects import Objects
from aisaac.msg import Status
import copy

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class NormalStartStrategyCalcurator(StrategyCalcuratorBase):
    """
    referee_branchがNORMAL_STARTの場合のCalcurator。
    """
    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase


        # コンテキストに保存されてる情報の拾い方(last_numberはworld_modelのコンストラクタでregisterしてある)
        last_number = strategy_context.get_last("last_number")
        stored_numbers = strategy_context.get_all("last_number")
        
        # InitialStaticStrategyを元に組み立てる
        self._dynamic_strategy.clone_from(self._static_strategies['initial'])
        
        # TODO: 色々計算

        # 生きてるロボットにだけ新たな指示を出す例
        # active_robot_ids = self._get_active_robot_ids()
        # status = Status()
        # for idx, robot_id in enumerate(active_robot_ids):
        #     if idx == 1:
        #         status.status = "defence1"
        #     elif idx == 2:
        #         status.status = "defence2"
        #     else:
        #         status.status = "move_linear"
        #     self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy

        # コンテキストに保存されてる情報の更新例
        strategy_context.update("last_number", last_number+1)
        # updateを2回以上1回のループの中で呼ぶと最後に呼ばれたものが保存される
        strategy_context.update("last_number", last_number+2)

        return result
