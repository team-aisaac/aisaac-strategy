#!/usr/bin/env python
# -*- coding: utf-8 -*-

from .strategy import DynamicStrategy
from .strategy_calcurator import StrategyCalcuratorBase
from aisaac.msg import Status
from common import functions

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class PenaltyAttack(StrategyCalcuratorBase):
    """
    referee_branchがDIRECT_FREE_ATTACKの場合のCalcurator。
    """
    def __init__(self, objects):
        self._objects = objects
        self._robot = objects.robot
        self._enemy = objects.enemy
        self._robot_ids = objects.get_robot_ids()
        self._enemy_ids = objects.get_enemy_ids()
        self._dynamic_strategy = DynamicStrategy()
        self._ball_params = objects.ball

        self.reset()

    def reset(self):
        self.history_who_has_a_ball = ["robots" for i in range(10)]

    def calcurate(self, strategy_context=None, should_wait=True):
        #print "penalty"

        if self._get_who_has_a_ball() == "free":
            pass
        elif self._get_who_has_a_ball() == "robots":
            self.history_who_has_a_ball.pop(0)
            self.history_who_has_a_ball.append("robots")
        else:
            self.history_who_has_a_ball.pop(0)
            self.history_who_has_a_ball.append("enemy")

        # if self.history_who_has_a_ball.count("enemy") > 5:
        #     strategy_context.update("penalty_finish", True, namespace="world_model")
        #     strategy_context.update("defence_or_attack", False, namespace="world_model")

        active_robot_ids = self._get_active_robot_ids()
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
                status.pid_goal_pos_x = 3.0
                status.pid_goal_pos_y = 1.5
            elif robot.get_role() == "RFW":
                status.status = "move_linear"
                status.pid_goal_pos_x = 3.0
                status.pid_goal_pos_y = -1.5
            else:
                status.status = "none"

            if  robot_id == active_robot_ids[0]:
                if should_wait:
                    status.status = "move_linear"
                    status.pid_goal_pos_x = 4.5
                    status.pid_goal_pos_y = 0.0
                else:
                    status.status = "penalty_shoot"

                    if functions.distance_btw_two_points(self._objects.ball.get_current_position(),
                                                         self._objects.get_robot_by_id(robot_id).get_current_position()) > 1.0:
                        strategy_context.update("penalty_finish", True, namespace="world_model")
                        strategy_context.update("defence_or_attack", False, namespace="world_model")


            self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy
        return result
