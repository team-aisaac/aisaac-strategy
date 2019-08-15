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
import functions
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

        if self.history_who_has_a_ball.count("enemy") > 5:
            strategy_context.update("direct_finish", True, namespace="world_model")
            strategy_context.update("defence_or_attack", False, namespace="world_model")
            # ここに入ると次からnormal_startが呼び出される

        ball_x, ball_y = self._objects.ball.get_current_position()

        if functions.distance_btw_two_points([ball_x, ball_y], [5.8, 4.3]) < 0.1:
            self.ckl_flg =  True
            self.ckr_flg =  False
        if functions.distance_btw_two_points([ball_x, ball_y], [5.8, -4.3]) < 0.1:
            self.ckl_flg =  False
            self.ckr_flg =  True

        if self._objects.get_ball_in_penalty_area() == "friend":
            # ボールが自陣ペナルティエリアに入っているときはGK
            result = self.calculate_goal_kick()
        elif self.ckl_flg:
            # ボールがCK左の位置にあるとき
            result = self.calculate_corner_kick_left()
        elif self.ckr_flg:
            #　ボールがCK右の位置にあるとき
            result = self.calculate_corner_kick_right()
        else:
            result = self.calculate_1()

        return result

    def calculate_goal_kick(self):
        active_robot_ids = self._get_active_robot_ids()
        for robot_id in active_robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "pass_chip"
                status.pass_target_pos_x = self._objects.get_robot_by_role("LFW").get_current_position()[0]
                status.pass_target_pos_y = self._objects.get_robot_by_role("LFW").get_current_position()[1]
            elif robot.get_role() == "LDF":
                status.status = "defence4"
            elif robot.get_role() == "RDF":
                status.status = "defence3"
            elif robot.get_role() == "LFW":
                #敵kickerとballの延長線上に移動
                status.status = "move_linear"
                status.pid_goal_pos_x = 1.0
                status.pid_goal_pos_y = 1.5
            elif robot.get_role() == "RFW":
                status.status = "move_linear"
                status.pid_goal_pos_x = 1.0
                status.pid_goal_pos_y = -1.5
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy
        return result

    def calculate_corner_kick_left(self):
        active_robot_ids = self._get_active_robot_ids()
        for robot_id in active_robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "keeper"
            elif robot.get_role() == "LDF":
                if self.passed_1_flg:
                    # パス完了した場合
                    status.status = "defence4"
                else:
                    # パス完了する前 パス先をposition1にする、パス完了したら完了flgを立てる
                    status.status = "pass"
                    status.pass_target_pos_x = self._objects.get_robot_by_role(self.lfw_or_rfw).get_current_position()[0]
                    status.pass_target_pos_y = self._objects.get_robot_by_role(self.lfw_or_rfw).get_current_position()[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_1_flg = True
                    if self.received_1_flg and self._objects.get_has_a_ball(robot_id) == False:
                        self.passed_1_flg = True
            elif robot.get_role() == "RDF":
                status.status = "defence3"
            elif robot.get_role() == "LFW":
                if self.position_2[1] > 0:
                    position = [self.position_2[0] + 1.2, self.position_2[1]]
                else:
                    position = self.position_3
                if self.passed_2_flg:
                    status.status = "move_linear"
                    # テキトーに移動
                    status.pid_goal_pos_x = 2.0
                    status.pid_goal_pos_y = 1.5
                if self.received_2_flg:
                    status.status = "shoot_left"
                    # status.status = "pass"
                    # status.pass_target_pos_x = 6.0
                    # status.pass_target_pos_y = 0.50
                    if self._objects.get_has_a_ball(robot_id) == False:
                        self.passed_2_flg = True
                else:
                    status.status = "receive"
                    status.pid_goal_pos_x = position[0]
                    status.pid_goal_pos_y = position[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_2_flg = True
            elif robot.get_role() == "RFW":
                if self.position_2[1] > 0:
                    position = self.position_3
                else:
                    position = [self.position_2[0] + 1.2, self.position_2[1]]
                if self.passed_3_flg:
                    status.status = "move_linear"
                    # テキトーに移動
                    status.pid_goal_pos_x = 2.0
                    status.pid_goal_pos_y = -1.5
                if self.received_3_flg:
                    status.status = "shoot_right"
                    # status.status = "pass"
                    # status.pass_target_pos_x = 6.0
                    # status.pass_target_pos_y = -0.50
                    if self._objects.get_has_a_ball(robot_id) == False:
                        self.passed_3_flg = True
                else:
                    status.status = "receive"
                    status.pid_goal_pos_x = position[0]
                    status.pid_goal_pos_y = position[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_3_flg = True
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy
        return result

    def calculate_corner_kick_right(self):
        active_robot_ids = self._get_active_robot_ids()
        for robot_id in active_robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "keeper"
            elif robot.get_role() == "LDF":
                status.status = "defence4"
            elif robot.get_role() == "RDF":
                if self.passed_1_flg:
                    # パス完了した場合
                    status.status = "defence3"
                else:
                    # パス完了する前 パス先をposition1にする、パス完了したら完了flgを立てる
                    status.status = "pass"
                    status.pass_target_pos_x = self._objects.get_robot_by_role(self.lfw_or_rfw).get_current_position()[0]
                    status.pass_target_pos_y = self._objects.get_robot_by_role(self.lfw_or_rfw).get_current_position()[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_1_flg = True
                    if self.received_1_flg and self._objects.get_has_a_ball(robot_id) == False:
                        self.passed_1_flg = True
            elif robot.get_role() == "LFW":
                if self.position_2[1] > 0:
                    position = [self.position_2[0] + 1.2, self.position_2[1]]
                else:
                    position = self.position_3
                if self.passed_2_flg:
                    status.status = "move_linear"
                    # テキトーに移動
                    status.pid_goal_pos_x = 2.0
                    status.pid_goal_pos_y = 1.5
                if self.received_2_flg:
                    status.status = "shoot_left"
                    # status.status = "pass"
                    # status.pass_target_pos_x = 6.0
                    # status.pass_target_pos_y = 0.50
                    if self._objects.get_has_a_ball(robot_id) == False:
                        self.passed_2_flg = True
                else:
                    status.status = "receive"
                    status.pid_goal_pos_x = position[0]
                    status.pid_goal_pos_y = position[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_2_flg = True
            elif robot.get_role() == "RFW":
                if self.position_2[1] > 0:
                    position = self.position_3
                else:
                    position = [self.position_2[0] + 1.2, self.position_2[1]]
                if self.passed_3_flg:
                    status.status = "move_linear"
                    # テキトーに移動
                    status.pid_goal_pos_x = 2.0
                    status.pid_goal_pos_y = -1.5
                if self.received_3_flg:
                    status.status = "shoot_right"
                    # status.status = "pass"
                    # status.pass_target_pos_x = 6.0
                    # status.pass_target_pos_y = -0.50
                    if self._objects.get_has_a_ball(robot_id) == False:
                        self.passed_3_flg = True
                else:
                    status.status = "receive"
                    status.pid_goal_pos_x = position[0]
                    status.pid_goal_pos_y = position[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_3_flg = True
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy
        return result
