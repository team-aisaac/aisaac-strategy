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
import  functions

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
        self._objects = objects
        self._robot = objects.robot
        self._enemy = objects.enemy
        self._robot_ids = objects.get_robot_ids()
        self._enemy_ids = objects.get_enemy_ids()
        self._dynamic_strategy = DynamicStrategy()
        self._ball_params = objects.ball

        self.reset()

    def reset(self):
        # パス先の決定
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
            [4.0, 0.5],
            [4.0, 0.6],
            [4.0, 0.4],
        ]
        self._lower_positions_3 = [
            [4.0, -0.5],
            [4.0, -0.6],
            [4.0, -0.4],
        ]

        self.passed_1_flg = False
        self.received_1_flg = False
        self.passed_2_flg = False
        self.received_2_flg = False
        self.passed_3_flg =False
        self.received_3_flg = False

        self.history_who_has_a_ball = ["robots" for i in range(10)]

        self.ball_position_x = self._objects.ball.get_current_position()[0]
        self.ball_position_y = self._objects.ball.get_current_position()[1]

        # ボールの位置が下側のときは受け手は上側にする、逆もまたしかり
        # position_1はFW_1が行く場所 (ボールが2mラインより左側にいるときに利用する、右側の時は使わない)
        # position2はFW2が行く場所
        # position3はFW1がボールを蹴ったあとに行く場所
        if self.ball_position_x <= 6.5:
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
            self.position_1_nearest_id = self._objects.get_robot_id_by_role("LFW")
            self.position_2_nearest_id = self._objects.get_robot_id_by_role("RFW")
            self.ball_position_nearest_id = self._objects.get_robot_id_by_role("LDF")
        else:
            self.position_1_nearest_id = self._objects.get_robot_id_by_role("RFW")
            self.position_2_nearest_id = self._objects.get_robot_id_by_role("LFW")
            self.ball_position_nearest_id = self._objects.get_robot_id_by_role("RDF")

        #self.position_2_nearest_id = self._objects.get_robot_ids_sorted_by_distance(self.position_2)[0]
        #self.ball_position_nearest_id = self._objects.get_robot_ids_sorted_by_distance([self.ball_position_x, self.ball_position_y])[0]
        self.GK_id = self._objects.get_robot_id_by_role("GK")
        self.LDF_id = self._objects.get_robot_id_by_role("LDF")
        self.RDF_id = self._objects.get_robot_id_by_role("RDF")
        self.last_calcurate = "calcurate_1"

        # ここから下はdirect用
        self.ckl_flg = False
        self.ckr_flg = False
        lr  = ["LFW", "RFW"]
        self.lfw_or_rfw = np.random.choice(lr)


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
            strategy_context.update("indirect_finish", True, namespace="world_model")
            strategy_context.update("defence_or_attack", False, namespace="world_model")

        result = self.calculate_1()
        return result



    def calculate_1(self, strategy_context=None):
        # print 'passed_1_flg:', self.passed_1_flg
        # print 'received_1_flg:', self.received_1_flg
        # print 'passed_2_flg:',self.passed_2_flg
        # print 'received_2_flg:', self.received_2_flg
        # print 'passed_3_flg:', self.passed_3_flg
        # print 'received_3_flg:', self.received_3_flg
        """ハーフラインより左側の戦略（の予定）"""
        ball_x, ball_y = self._objects.ball.get_current_position()
        active_enemy_ids = self._get_active_enemy_ids()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        for robot_id in self._robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)

            if self.ball_position_nearest_id == robot_id:
                if self.passed_1_flg:
                    # パス完了した場合
                    if self._objects.robot[robot_id].get_role() == "LDF":
                        status.status = "defence4"
                    else:
                        status.status = "defence3"
                else:
                    # パス完了する前 パス先をposition1にする、パス完了したら完了flgを立てる
                    status.status = "pass"
                    status.pass_target_pos_x = self._objects.get_robot_by_id(self.position_1_nearest_id).get_current_position()[0]
                    status.pass_target_pos_y = self._objects.get_robot_by_id(self.position_1_nearest_id).get_current_position()[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_1_flg = True
                    if self.received_1_flg and self._objects.get_has_a_ball(robot_id) == False:
                        self.passed_1_flg = True

            elif robot_id == self.position_1_nearest_id:
                if self.received_3_flg:
                    #シュートを打つ子がボールを受け取ったあとはこの機体もシュート体制に入る(おこぼれを狙うイメージ)
                    #status.status = "pass"
                    if self.position_2[1] > 0.:
                        status.status = "shoot_left"
                        # status.pass_target_pos_x = 6.0
                        # status.pass_target_pos_y = 0.55
                    else:
                        status.status = "shoot_right"
                        # status.pass_target_pos_x = 6.0
                        # status.pass_target_pos_y = -0.55

                elif self.passed_2_flg:
                    #　自分がパスしたあとはゴール前のposition_3に移動
                    status.status = "move_linear"
                    status.pid_goal_pos_x = self.position_1[0]
                    status.pid_goal_pos_y = self.position_1[1]
                    status.pid_goal_theta = 0.

                elif self.received_2_flg:
                    # レシーブしたあとはパスに切り替え
                    status.status = "pass"
                    status.pass_target_pos_x = self._objects.get_robot_by_id(self.position_2_nearest_id).get_current_position()[0]
                    status.pass_target_pos_y = self._objects.get_robot_by_id(self.position_2_nearest_id).get_current_position()[1]
                    if self._objects.get_has_a_ball(self.position_2_nearest_id, threshold=0.20) == True:
                        self.passed_2_flg
                else:
                    status.status = "receive"
                    status.pid_goal_pos_x = self.position_1[0]
                    status.pid_goal_pos_y = self.position_1[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_2_flg = True

            elif robot_id == self.position_2_nearest_id:
                if self.passed_3_flg:
                    status.status = "move_linear"
                    if nearest_enemy_id != None:
                        status.pid_goal_pos_x, status.pid_goal_pos_y = functions.calculate_internal_dividing_point(self._enemy[nearest_enemy_id].get_current_position()[0], self._enemy[nearest_enemy_id].get_current_position()[1], self._ball_params.get_current_position()[0], self._ball_params.get_current_position()[1], functions.distance_btw_two_points(self._enemy[nearest_enemy_id].get_current_position(), self._ball_params.get_current_position()) + 0.55, -0.55)
                        status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - self._robot[3].get_current_position()[1]) , (self._ball_params.get_current_position()[0] - self._robot[2].get_current_position()[0]) )
                elif self.received_3_flg:
                    # status.status = "pass"
                    if self.position_3[1] > 0.:
                        status.status = "shoot_left"
                        # status.pass_target_pos_x = 6.0
                        # status.pass_target_pos_y = 0.55
                    else:
                        status.status = "shoot_right"
                        # status.pass_target_pos_x = 6.0
                        # status.pass_target_pos_y = -0.55
                    if self._objects.get_has_a_ball(robot_id) == False:
                        self.passed_3_flg = True
                else:
                    status.status = "receive"
                    status.pid_goal_pos_x = self.position_2[0]
                    status.pid_goal_pos_y = self.position_2[1]
                    if self._objects.get_has_a_ball(robot_id):
                        self.received_3_flg = True

            elif robot_id == self.GK_id:
                status.status = "keeper"

            elif robot.get_role() == "LDF":
                status.status = "defence4"
            elif robot.get_role() == "RDF":
                status.status = "defence3"
            else:
                print 'error'

            self._dynamic_strategy.set_robot_status(robot_id, status)
        return self._dynamic_strategy

    def calculate_defense(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase

        active_robot_ids = self._get_active_robot_ids()
        active_enemy_ids = self._get_active_enemy_ids()
        status = Status()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        for robot_id in active_robot_ids:
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
                if nearest_enemy_id != None:
                    status.pid_goal_pos_x, status.pid_goal_pos_y = functions.calculate_internal_dividing_point(self._enemy[nearest_enemy_id].get_current_position()[0], self._enemy[nearest_enemy_id].get_current_position()[1], self._ball_params.get_current_position()[0], self._ball_params.get_current_position()[1], functions.distance_btw_two_points(self._enemy[nearest_enemy_id].get_current_position(), self._ball_params.get_current_position()) + 0.55, -0.55)
                    status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - self._robot[3].get_current_position()[1]) , (self._ball_params.get_current_position()[0] - self._robot[2].get_current_position()[0]) )
            elif robot.get_role() == "RFW":
                #フリーで最もゴールに近い敵idを返す
                status.status = "move_linear"
                free_enemy_id = self._get_free_enemy_id(4, nearest_enemy_id)
                status.pid_goal_pos_x = (self._ball_params.get_current_position()[0] + self._enemy[free_enemy_id].get_current_position()[0]) / 2
                status.pid_goal_pos_y = (self._ball_params.get_current_position()[1] + self._enemy[free_enemy_id].get_current_position()[1]) / 2
                status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - self._robot[4].get_current_position()[1]) , (self._ball_params.get_current_position()[0] - self._robot[4].get_current_position()[0]) )
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)


        result = self._dynamic_strategy
        return result
