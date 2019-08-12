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
import rospy

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class NormalStartStrategyCalcurator(StrategyCalcuratorBase):
    def __init__(self, objects):
        super(NormalStartStrategyCalcurator, self).__init__(objects)
        self._receiver_id = None
        self._kicker_id = None

    def _reset_state(self):
        pass

    def _pass_and_shoot(self, pass_positions, strategy_context):
        pass

    def _attack_strategy1(self, strategy_context=None):
        pass_positions = [
            [2.0, 2.0],
            [-2.0, -2.0],
            [6.0, 0.0],
        ]

        last_state = strategy_context.get_last("normal_strat_state")

        self._reset_state()

        area = self._objects.robot[0].size_r + 0.3

        ball_pos = self._objects.ball.get_current_position()
        # if self._receiver_id is None:
        target_pos = pass_positions[last_state]
        # else:
        #     # パスの目標ではなく、受け取る側のロボットが実際にリーチした場所を目標とする
        #     target_pos = self._objects.robot[self._receiver_id].get_current_position()

        distance = functions.distance_btw_two_points(ball_pos, target_pos)

        cur_state = last_state

        if distance < area:
            if len(pass_positions) > cur_state:
                cur_state = cur_state + 1

        is_shoot = False
        if len(pass_positions) - 1 == cur_state:
            is_shoot = True

        should_reset = False
        if len(pass_positions) == cur_state:
            cur_state = cur_state - 1
            should_reset = True

        # print("State:"+str(cur_state))

        # InitialStaticStrategyを元に組み立てる
        self._dynamic_strategy.clone_from(self._static_strategies['initial'])

        # 生きてるロボットにだけ新たな指示を出す例
        active_robot_ids = self._get_active_robot_ids()

        not_assigned_robot_ids = active_robot_ids


        # Defence系を先にアサイン
        not_assigned_ops = ["keeper", "defence3", "defence4"]

        # for文で消えないようコピー
        sorted_not_assigned_robot_ids = sorted(not_assigned_robot_ids, reverse=True)
        for robot_id in sorted_not_assigned_robot_ids:
            if not_assigned_ops == []:
                break
            status = Status()
            status.status = not_assigned_ops[0]

            self._dynamic_strategy.set_robot_status(robot_id, status)

            not_assigned_ops.pop(0)
            not_assigned_robot_ids.remove(robot_id)

        robots_near_to_ball = self._objects.get_robot_ids_sorted_by_distance_to_ball(not_assigned_robot_ids)

        for idx, robot_id in enumerate(robots_near_to_ball):
            status = Status()
            if idx == 0:
                status.status = "pass"
                if self._receiver_id is not None:
                    receiver_pos = self._objects.robot[self._receiver_id].get_current_position()
                    receiver_area = 1.0
                    if functions.distance_btw_two_points(pass_positions[cur_state], receiver_pos) > receiver_area:
                        status.status = "prepare_pass"

                self._kicker_id = robot_id
                status.pass_target_pos_x = pass_positions[cur_state][0]
                status.pass_target_pos_y = pass_positions[cur_state][1]
                not_assigned_robot_ids.remove(robot_id)
            elif idx == 1:
                if is_shoot:
                    status.status = "stop"
                    self._receiver_id = None
                else:
                    status.status = "receive"
                    self._receiver_id = robot_id
                status.pass_target_pos_x = pass_positions[cur_state][0]
                status.pass_target_pos_y = pass_positions[cur_state][1]
                not_assigned_robot_ids.remove(robot_id)

            self._dynamic_strategy.set_robot_status(robot_id, status)

        # self._dynamic_strategy.clone_from(self._static_strategies['initial'])
        if should_reset:
            # シュートしたらリセット
            strategy_context.update("normal_strat_state", 0)
            self._receiver_id = None
        else:
            strategy_context.update("normal_strat_state", cur_state)

        result = copy.deepcopy(self._dynamic_strategy)
        return result

    """
    referee_branchがNORMAL_STARTの場合のCalcurator。
    """
    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase
        if True:
            result = self._attack_strategy1(strategy_context)

        return result
