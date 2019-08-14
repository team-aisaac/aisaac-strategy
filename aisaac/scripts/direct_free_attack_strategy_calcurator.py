#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from abc import ABCMeta, abstractmethod

from strategy import StrategyBase, InitialStaticStrategy, StopStaticStrategy, DynamicStrategy
from strategy_calcurator import StrategyCalcuratorBase
from context import StrategyContext
from indirect_free_attack_strategy_calcurator import IndirectFreeAttack
from objects import Objects
from aisaac.msg import Status
import copy

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class DirectFreeAttack(IndirectFreeAttack):
    """
    referee_branchがDIRECT_FREE_ATTACKの場合のCalcurator。
    """
    def __init__(self, objects):
        super(DirectFreeAttack, self).__init__(objects)

    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase

        if self._get_who_has_a_ball() == "free":
            pass
        elif self._get_who_has_a_ball() == "robots":
            self.history_who_has_a_ball.pop(0)
            self.history_who_has_a_ball.append("robots")
        else:
            self.history_who_has_a_ball.pop(0)
            self.history_who_has_a_ball.append("enemy")

        if : # TODO  ballが味方ペナルティエリアに入ってるか

        elif functions.distance_btw_two_points([ball_x, ball_y], [6., 4.5]) < 0.1 or functions.distance_btw_two_points([ball_x, ball_y], [6., -4.5]) < 0.1:
            result = self.calculate_corner_kick()

        if self.history_who_has_a_ball.count("enemy") > 5:
            strategy_context.update("direct_finish", True, namespace="world_model")
            strategy_context.update("defence_or_attack", False, namespace="world_model")


        result = self.calculate_1()
        return result

    def calculate_goal_kick(self):
        for robot_id in active_robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "pass_chip"
                status.pass_target_pos_x = self._objects.get_robot_by_role("LFW").get_current_position()[0]
                status.pass_target_pos_y = self._objects.get_robot_by_role("LFW").get_current_position()[1]
            elif robot.get_role() == "LDF":
                status.status = "defence1"
            elif robot.get_role() == "RDF":
                status.status = "defence2"
            elif robot.get_role() == "LFW":
                #敵kickerとballの延長線上に移動
                status.status = "move_linear"
                status.pid_goal_pos_x = 2.0
                status.pid_goal_pos_y = 1.5
            elif robot.get_role() == "RFW":
                status.status = "move_linear"
                status.pid_goal_pos_x = 2.0
                status.pid_goal_pos_y = -1.5
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy
        return result

    def calculate_corner_kick_left(self):
        for robot_id in active_robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "pass_chip"
                status.pass_target_pos_x = self._objects.get_robot_by_role("LFW").get_current_position()[0]
                status.pass_target_pos_y = self._objects.get_robot_by_role("LFW").get_current_position()[1]
            elif robot.get_role() == "LDF":
                status.status = ""
                # Todo

            elif robot.get_role() == "RDF":
                status.status = "defence2"
            elif robot.get_role() == "LFW":
                #敵kickerとballの延長線上に移動
                status.status = "move_linear"
                status.pid_goal_pos_x = 2.0
                status.pid_goal_pos_y = 1.5
            elif robot.get_role() == "RFW":
                status.status = "move_linear"
                status.pid_goal_pos_x = 2.0
                status.pid_goal_pos_y = -1.5
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy
        return result
