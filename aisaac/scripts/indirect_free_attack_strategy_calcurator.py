#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
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

    今回は決め打ちで作る
    1. ボールの位置がハーフラインより手前にあるとき
        Defenderの一番近い子がAttacker1(positions_1)にボールを蹴る　Defenderは蹴ったあとは守備モード
        Attacker1がAttacker2(positions_2)にボールを蹴る　Attacker1はGoal側に走っておき(positions_3)、passモードにしておく(to Goal)
        Attacker2はそのままシュート
    2. ボールの位置がハーフラインより奥側にあるとき
        Attacker1がAttacker2(positions_2)にボールを蹴る Attacker1はGoal側に走っておき(positions_3)、passモードにしておく(to Goal)
        Attacker2はそのままシュート

    """
    def __init__(self, objects):
        self.objects = objects
        self.friend = objects.robot
        self._robot_ids = objects.get_robot_ids()
        self._dynamic_strategy = DynamicStrategy()

        self._upper_positions_1 = [
            [3.5, 2.5],
            [3.5, 2.0],
            [3.0, 2.0],
        ]
        self._lower_positions_1 = [
            [3.5, -2.5],
            [3.5, -2.0],
            [3.0, -2.0],
        ]

        self._upper_positions_2 = [
            [3.5, 2.5],
            [3.5, 2.0],
            [3.0, 2.0],
        ]
        self._lower_positions_2 = [
            [3.5, -2.5],
            [3.5, -2.0],
            [3.0, -2.0],
        ]

        self._upper_positions_3 = [
            [5.0, 0.5],
            [5.0, 0.6],
            [5.0, 0.4],
        ]
        self._lower_positions_3 = [
            [5.0, -0.5],
            [5.0, -0.6],
            [5.0, -0.4],
        ]

        self.passed_1_flg = False
        self.received_1_flg = False
        self.passed_2_flg = False
        self.received_2_flg = False
        self.passed_3_flg = False
        self.received_3_flg = False

        self.ball_position_x = self.objects.ball.get_current_position()[0]
        self.ball_position_y = self.objects.ball.get_current_position()[1]

        self.reset()

    def reset(self):
        self.passed_1_flg = False
        self.received_1_flg = False
        self.passed_2_flg = False
        self.received_2_flg = False
        self.passed_3_flg = False
        self.received_3_flg = False

        self.ball_position_x = self.objects.ball.get_current_position()[0]
        self.ball_position_y = self.objects.ball.get_current_position()[1]

        # ボールの位置が下側のときは受け手は上側にする、逆もまたしかり
        # position_1はFW_1が行く場所 (ボールがハーフラインより左側にいるときに利用する、右側の時は使わない)
        # position2はFW2が行く場所
        # position3はFW1がボールを蹴ったあとに行く場所
        if self.ball_position_x <= 0.:
            if self.ball_position_y <= 0.:
                positions_1 = self._upper_positions_1
                positions_2 = self._lower_positions_2
                positions_3 = self._upper_positions_3
            else:
                positions_1 = self._lower_positions_1
                positions_2 = self._upper_positions_2
                positions_3 = self._lower_positions_3
        else:
            if self.ball_position_y <= 0.:
                positions_1 = False
                positions_2 = self._upper_positions_2
                positions_3 = self._lower_positions_3
            else:
                positions_1 = False
                positions_2 = self._lower_positions_2
                positions_3 = self._upper_positions_3

        if positions_1:
            choice_index = np.random.choice(len(positions_1))
            self.position_1 = positions_1[choice_index]
        else:
            self.position_1 = False
        choice_index = np.random.choice(len(positions_2))
        self.position_2 = positions_2[choice_index]
        choice_index = np.random.choice(len(positions_3))
        self.position_3 = positions_3[choice_index]

        # position1 ~ position3の位置を決定
        if self.position_1 and self.position_1[0] >= 0.:
            self.position_1_nearest_id = self.objects.get_robot_id_by_role("LFW")
            self.position_2_nearest_id = self.objects.get_robot_id_by_role("RFW")
            self.ball_position_nearest_id = self.objects.get_robot_id_by_role("LDF")
        else:
            self.position_1_nearest_id = self.objects.get_robot_id_by_role("RFW")
            self.position_2_nearest_id = self.objects.get_robot_id_by_role("LFW")
            self.ball_position_nearest_id = self.objects.get_robot_id_by_role("RDF")

        #self.position_2_nearest_id = self.objects.get_robot_ids_sorted_by_distance(self.position_2)[0]
        #self.ball_position_nearest_id = self.objects.get_robot_ids_sorted_by_distance([self.ball_position_x, self.ball_position_y])[0]
        self.GK_id = self.objects.get_robot_id_by_role("GK")
        self.LDF_id = self.objects.get_robot_id_by_role("LDF")
        self.RDF_id = self.objects.get_robot_id_by_role("RDF")

    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase

        result = self.calculate_1()
        return result

    def calculate_1(self, strategy_context=None):
        """ハーフラインより左側の戦略"""
        ball_x, ball_y = self.objects.ball.get_current_position()
        for robot_id in self._robot_ids:
            status = Status()
            robot = self.objects.get_robot_by_id(robot_id)

            if self.ball_position_nearest_id == robot_id:
                if self.passed_1_flg:
                    # パス完了した場合
                    status.status = "defence3"
                else:
                    # パス完了する前 パス先をposition1にする、パス完了したら完了flgを立てる
                    status.status = "pass"
                    status.pass_target_pos_x = self.objects.get_robot_by_id(self.position_1_nearest_id).get_current_position()[0]
                    status.pass_target_pos_y = self.objects.get_robot_by_id(self.position_1_nearest_id).get_current_position()[1]
                    if robot.has_a_ball(ball_x, ball_y):
                        self.received_1_flg = True
                    if self.received_1_flg and robot.has_a_ball(ball_x, ball_y) == False:
                        self.passed_1_flg = True

            elif robot_id == self.position_1_nearest_id:
                if self.received_3_flg:
                    #シュートを打つ子がボールを受け取ったあとはこの機体もシュート体制に入る(おこぼれを狙うイメージ)
                    status.status = "pass"
                    if self.position_2[1] > 0.:
                        status.pass_target_pos_x = 6.0
                        status.pass_target_pos_y = 0.55
                    else:
                        status.pass_target_pos_x = 6.0
                        status.pass_target_pos_y = -0.55

                elif self.passed_2_flg:
                    #　自分がパスしたあとはゴール前のposition_3に移動
                    status.status = "move_linear"
                    status.pid_goal_pos_x = self.position_1[0]
                    status.pid_goal_pos_y = self.position_1[1]
                    status.pid_goal_theta = 0.

                elif self.received_2_flg:
                    # レシーブしたあとはパスに切り替え
                    status.status = "pass"
                    status.pass_target_pos_x = self.objects.get_robot_by_id(self.position_2_nearest_id).get_current_position()[0]
                    status.pass_target_pos_y = self.objects.get_robot_by_id(self.position_2_nearest_id).get_current_position()[1]
                    if robot.has_a_ball(ball_x, ball_y):
                        self.passed_2_flg = True
                else:
                    status.status = "receive"
                    #  TODO
                    status.pass_target_pos_x = self.position_1[0]
                    status.pass_target_pos_y = self.position_1[1]
                    if robot.has_a_ball(ball_x, ball_y):
                        self.received_2_flg = True

            elif robot_id == self.position_2_nearest_id:
                if self.passed_3_flg:
                    # TODO 防御を書く
                    None

                elif self.received_3_flg:
                    status.status = "pass"
                    if self.position_3[1] > 0.:
                        status.pass_target_pos_x = 6.0
                        status.pass_target_pos_y = 0.55
                    else:
                        status.pass_target_pos_x = 6.0
                        status.pass_target_pos_y = -0.55
                else:
                    status.status = "receive"
                    #  TODO
                    status.pass_target_pos_x = self.position_2[0]
                    status.pass_target_pos_y = self.position_2[1]
                    if robot.has_a_ball(ball_x, ball_y):
                        self.received_3_flg = True

            elif robot_id == self.GK_id:
                status.status = "keeper"

            else:
                if robot.get_role() == "LDF":
                    status.status = "defence3"
                elif robot.get_role() == "RDF":
                    status.status = "defence4"

            self._dynamic_strategy.set_robot_status(robot_id, status)
        return self._dynamic_strategy
