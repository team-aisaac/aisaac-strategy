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
import numpy as np

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class NormalStartStrategyCalcurator(StrategyCalcuratorBase):
    def __init__(self, objects):
        super(NormalStartStrategyCalcurator, self).__init__(objects)
        self._receiver_id = None
        self._kicker_id = None
        self._robots_near_to_ball = None

        self._prepare_pass = False
        self._prepare_pass_start_time = rospy.Time.now()

        self._pass_positions = None
        self._random_fake_position = self._generate_pass_position()

    def _generate_pass_position(self, offset=0.0):
        """
        Parameters
        ----------
        offset: float このxより前側にパスコースを作る
        """
        tmp_x = offset + (4.8 - offset) * np.random.rand()
        tmp_y = 8.0 * (-0.5 + np.random.rand())
        return tmp_x, tmp_y

    def _generate_pass_positions(self):
        length_pass_stage = np.random.choice([0,1,2])
        pass_positions = []
        tmp_pass_pos = (0.0, 0.0)
        for i in range(length_pass_stage):
            tmp_pass_pos = self._generate_pass_position(tmp_pass_pos[0])
            pass_positions.append(tmp_pass_pos)
        pass_positions.append([6.0, 0.0])

        rospy.loginfo("パス経路："+str(pass_positions))

        return pass_positions

    def _attack_strategy1(self, strategy_context=None):

        #ball喪失時、防御へ移行
        if self._get_who_has_a_ball() == "enemy":
                strategy_context.update("defence_or_attack", False, namespace="world_model")

        if self._pass_positions is None:
            self._pass_positions = self._generate_pass_positions()

        pass_positions = self._pass_positions

        succeeded_area = self._objects.robot[0].size_r + 0.62

        last_state = strategy_context.get_last("normal_strat_state", namespace="normal_strat")

        ball_pos = self._objects.ball.get_current_position()

        if self._receiver_id is None:
            target_pos = pass_positions[last_state]
        else:
            # パスの目標ではなく、受け取る側のロボットが実際にリーチした場所を目標とする
            target_pos = self._objects.get_robot_by_id(self._receiver_id).get_current_position()

        distance = functions.distance_btw_two_points(ball_pos, target_pos)
        # print("Receiver: "+ str(self._receiver_id) +" Distance:"+str(distance))

        cur_state = last_state

        # 目標位置近くにボールが行ったら次のステートへ
        change_state = False
        if distance < succeeded_area:
            cur_state = cur_state + 1
            self._random_fake_position = self._generate_pass_position()
            change_state = True

        # 最後のpass_positionsだったらシュート
        is_shoot = False
        if len(pass_positions) - 1 == cur_state:
            is_shoot = True

        # 最後の一つ前のpass_positionsだったらシュートのためのレシーブ
        pre_shoot = False
        if len(pass_positions) - 2 == cur_state:
            pre_shoot = True

        # シュートが終了したらリセット
        should_reset = False
        if len(pass_positions) == cur_state:
            cur_state = cur_state - 1
            should_reset = True

        # print("State:"+str(cur_state))

        # InitialStaticStrategyを元に組み立てる
        self._dynamic_strategy.clone_from(self._static_strategies['initial'])

        not_assigned_robot_ids = self._get_active_robot_ids()
        tmp_not_assigned_robot_ids = copy.deepcopy(not_assigned_robot_ids)

        # Defence系をアサイン
        for robot_id in tmp_not_assigned_robot_ids:

            status = Status()

            # 2台以下ならとりあえずGKをアサイン
            if len(not_assigned_robot_ids) <= 2:
                status.status = "keeper"
                self._dynamic_strategy.set_robot_status(robot_id, status)
                not_assigned_robot_ids.remove(robot_id)
                break

            role = self._objects.get_robot_by_id(robot_id).get_role()
            if role == 'GK':
                status.status = "keeper"
                self._dynamic_strategy.set_robot_status(robot_id, status)
                not_assigned_robot_ids.remove(robot_id)
            elif role == 'LDF':
                status.status = "defence4"
                self._dynamic_strategy.set_robot_status(robot_id, status)
                not_assigned_robot_ids.remove(robot_id)
            elif role == 'RDF':
                status.status = "defence3"
                self._dynamic_strategy.set_robot_status(robot_id, status)
                not_assigned_robot_ids.remove(robot_id)


        # ステートが変わるときに近い順を更新
        if self._robots_near_to_ball is None or change_state:
            self._robots_near_to_ball = self._objects.get_robot_ids_sorted_by_distance_to_ball(not_assigned_robot_ids)


        if len(self._robots_near_to_ball) >= 2: # 2台以上攻撃に割ける場合
            # 残りをボールに近い順にアサイン
            for idx, robot_id in enumerate(self._robots_near_to_ball):
                status = Status()
                if idx == 0:
                    if is_shoot:
                        status.status = "shoot"
                    else:
                        status.status = "pass"
                    if self._receiver_id is not None:
                        receiver_pos = self._objects.get_robot_by_id(self._receiver_id).get_current_position()
                        receiver_area = 1.0
                        if functions.distance_btw_two_points(pass_positions[cur_state], receiver_pos) > receiver_area:
                            if not self._prepare_pass:
                                self._prepare_pass_start_time = rospy.Time.now()
                                self._prepare_pass = True
                            if (rospy.Time.now() - self._prepare_pass_start_time).to_sec() > 1.0:
                                status.status = "pass"
                            else:
                                status.status = "prepare_pass"
                        else:
                            self._prepare_pass = False

                    self._kicker_id = robot_id
                    status.pass_target_pos_x = pass_positions[cur_state][0]
                    status.pass_target_pos_y = pass_positions[cur_state][1]
                    not_assigned_robot_ids.remove(robot_id)
                elif idx == 1:
                    if is_shoot:
                        status.status = "receive"
                        if self._kicker_id is not None:
                            # 適当な位置に移動
                            x, y = self._random_fake_position
                            status.pid_goal_pos_x = x
                            status.pid_goal_pos_y = y
                        self._receiver_id = None
                    else:
                        # if pre_shoot:
                        #     status.status = "receive_direct_shoot"
                        # else:
                        #     status.status = "receive"
                        status.status = "receive"
                        self._receiver_id = robot_id
                        if not should_reset:
                            status.pass_target_pos_x = pass_positions[cur_state+1][0]
                            status.pass_target_pos_y = pass_positions[cur_state+1][1]
                        status.pid_goal_pos_x = pass_positions[cur_state][0]
                        status.pid_goal_pos_y = pass_positions[cur_state][1]
                    not_assigned_robot_ids.remove(robot_id)

                self._dynamic_strategy.set_robot_status(robot_id, status)

        else: # 1台以下の場合あまりがいればとりあえずゴールにシュート
            for robot_id in self._robots_near_to_ball:
                status = Status()
                status.status = "shoot"
                status.pass_target_pos_x = pass_positions[-1][0]
                status.pass_target_pos_y = pass_positions[-1][1]
                self._dynamic_strategy.set_robot_status(robot_id, status)

        # self._dynamic_strategy.clone_from(self._static_strategies['initial'])
        if should_reset:
            # シュートしたらリセット
            strategy_context.update("normal_strat_state", 0, namespace="normal_strat")
            self._receiver_id = None
            self._pass_positions = None
        else:
            strategy_context.update("normal_strat_state", cur_state, namespace="normal_strat")

        result = self._dynamic_strategy
        return result


    def _defence_strategy(self, strategy_context=None):

        #ball取得時、攻撃へ移行
        if self._get_who_has_a_ball() == "robots":
                strategy_context.update("defence_or_attack", True, namespace="world_model")

        active_robot_ids = self._get_active_robot_ids()
        active_enemy_ids = self._get_active_enemy_ids()
        status = Status()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        for idx, robot_id in enumerate(active_robot_ids):
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "keeper"
            elif robot.get_role() == "LDF":
                status.status = "defence4"
            elif robot.get_role() == "RDF":
                status.status = "defence3"
            elif robot.get_role() == "LFW":
                #ballの位置に移動
                status.status = "move_linear"
                status.pid_goal_pos_x = self._ball_params.get_current_position()[0]
                status.pid_goal_pos_y = self._ball_params.get_current_position()[1]
                status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]) )
            elif robot.get_role() == "RFW":
                #フリーで最もゴールに近い敵idを返す
                status.status = "move_linear"
                free_enemy_id = self._get_free_enemy_id(4, nearest_enemy_id)
                status.pid_goal_pos_x = (self._ball_params.get_current_position()[0] + self._enemy[free_enemy_id].get_current_position()[0]) / 2
                status.pid_goal_pos_y = (self._ball_params.get_current_position()[1] + self._enemy[free_enemy_id].get_current_position()[1]) / 2
                status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]) )
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)


        result = self._dynamic_strategy
        return result
    """
    referee_branchがNORMAL_STARTの場合のCalcurator。
    """
    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase
        if strategy_context.get_last("defence_or_attack", namespace="world_model"):
            result = self._attack_strategy1(strategy_context)
        else:
            result = self._defence_strategy(strategy_context)

        return result


class NormalStartKickOffStrategyCalcurator(StrategyCalcuratorBase):
    def __init__(self, objects):
        super(NormalStartKickOffStrategyCalcurator, self).__init__(objects)
        self._receiver_id = None
        self._kicker_id = None

        self._prepare_pass = False
        self._prepare_pass_start_time = rospy.Time.now()

    def calcurate(self, strategy_context=None):
        pass_pos = [-0.3, -2.0]
        succeeded_area = self._objects.robot[0].size_r + 0.3

        ball_pos = self._objects.ball.get_current_position()
        target_pos = pass_pos
        distance = functions.distance_btw_two_points(ball_pos, target_pos)

        if distance < succeeded_area:
            strategy_context.update("kickoff_complete", True, namespace="world_model")
            strategy_context.update("defence_or_attack", True, namespace="world_model")

        self._dynamic_strategy.clone_from(self._static_strategies['initial'])

        not_assigned_robot_ids = self._get_active_robot_ids()
        tmp_not_assigned_robot_ids = copy.deepcopy(not_assigned_robot_ids)

        for robot_id in tmp_not_assigned_robot_ids:

            if len(not_assigned_robot_ids) <= 2:
                break

            status = Status()
            role = self._objects.get_robot_by_id(robot_id).get_role()
            if role == 'GK':
                status.status = "keeper"
                self._dynamic_strategy.set_robot_status(robot_id, status)
                not_assigned_robot_ids.remove(robot_id)
            elif role == 'LDF':
                status.status = "defence4"
                self._dynamic_strategy.set_robot_status(robot_id, status)
                not_assigned_robot_ids.remove(robot_id)
            elif role == 'RDF':
                status.status = "defence3"
                self._dynamic_strategy.set_robot_status(robot_id, status)
                not_assigned_robot_ids.remove(robot_id)

        for idx, robot_id in enumerate(not_assigned_robot_ids):
            status = Status()
            if idx == 0:
                if len(not_assigned_robot_ids) == 1:
                    status.status = "shoot"
                else:
                    status.status = "pass"

                if self._receiver_id is not None:
                    receiver_pos = self._objects.get_robot_by_id(self._receiver_id).get_current_position()
                    receiver_area = 1.0
                    if functions.distance_btw_two_points(pass_pos, receiver_pos) > receiver_area:
                        if not self._prepare_pass:
                            self._prepare_pass_start_time = rospy.Time.now()
                            self._prepare_pass = True
                        if (rospy.Time.now() - self._prepare_pass_start_time).to_sec() > 3.0:
                            status.status = "pass"
                        else:
                            status.status = "prepare_pass"
                    else:
                        self._prepare_pass = False

                self._kicker_id = robot_id
                status.pass_target_pos_x = pass_pos[0]
                status.pass_target_pos_y = pass_pos[1]
            elif idx == 1:
                status.status = "receive"
                self._receiver_id = robot_id
                status.pid_goal_pos_x = pass_pos[0]
                status.pid_goal_pos_y = pass_pos[1]
            self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy
        return result

class NormalStartKickOffDefenceStrategyCalcurator(StrategyCalcuratorBase):
    """
    referee_branchがKICKOFF_DEFENCEの場合のCalcurator。
    """
    def __init__(self, objects):
        self._robot = objects.robot
        self._enemy = objects.enemy
        self._robot_ids = objects.get_robot_ids()
        self._enemy_ids = objects.get_enemy_ids()
        self._ball_params = objects.ball
        self._dynamic_strategy = DynamicStrategy()
        self._objects = objects

    def calcurate(self, strategy_context=None, referee_branch="KICKOFF_DEFENCE"):
        # type: (StrategyContext) -> StrategyBase

        if referee_branch == "NORMAL_START":
            if self._detect_enemy_kick(strategy_context):
                strategy_context.update("enemy_kick", True, namespace="world_model")
                strategy_context.update("defence_or_attack", False, namespace="world_model")
        else:
            strategy_context.update("placed_ball_position", self._ball_params.get_current_position(), namespace="world_model")

        active_robot_ids = self._get_active_robot_ids()
        active_enemy_ids = self._get_active_enemy_ids()
        status = Status()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        for idx, robot_id in enumerate(active_robot_ids):
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
                    status.pid_goal_pos_x, status.pid_goal_pos_y = functions.calculate_internal_dividing_point(self._enemy[nearest_enemy_id].get_current_position()[0], self._enemy[nearest_enemy_id].get_current_position()[1], self._ball_params.get_current_position()[0], self._ball_params.get_current_position()[1], functions.distance_btw_two_points(self._enemy[nearest_enemy_id].get_current_position(), self._ball_params.get_current_position()) + 0.8, -0.8)
                    status.pid_goal_theta = math.atan2((self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]))
                    if status.pid_goal_pos_x >= -0.2 and status.pid_goal_pos_y >= 0:
                        status.pid_goal_pos_x = -0.2
                        status.pid_goal_pos_y = 0.8
                    elif status.pid_goal_pos_x >= -0.2 and status.pid_goal_pos_y < 0:
                        status.pid_goal_pos_x = -0.2
                        status.pid_goal_pos_y = -0.8
            elif robot.get_role() == "RFW":
                #固定位置
                status.status = "move_linear"
                free_enemy_id = self._get_free_enemy_id(4, nearest_enemy_id)
                status.pid_goal_pos_x = -3.
                status.pid_goal_pos_y = 0.
                status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]) )
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)


        result = self._dynamic_strategy
        return result

class NormalStartPenaltyDefenceStrategyCalcurator(StrategyCalcuratorBase):
    """
    referee_branchがPENALTY_DEFENCEの場合のCalcurator。
    """
    def __init__(self, objects):
        self._robot = objects.robot
        self._enemy = objects.enemy
        self._robot_ids = objects.get_robot_ids()
        self._enemy_ids = objects.get_enemy_ids()
        self._ball_params = objects.ball
        self._dynamic_strategy = DynamicStrategy()
        self._objects = objects

    def calcurate(self, strategy_context=None, referee_branch="PENALTY_DEFENCE"):
        # type: (StrategyContext) -> StrategyBase

        if referee_branch == "NORMAL_START":
            if self._detect_enemy_kick(strategy_context):
                strategy_context.update("enemy_kick", True, namespace="world_model")
                strategy_context.update("defence_or_attack", False, namespace="world_model")
        else:
            strategy_context.update("placed_ball_position", self._ball_params.get_current_position(), namespace="world_model")

        active_robot_ids = self._get_active_robot_ids()
        active_enemy_ids = self._get_active_enemy_ids()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        for robot_id in active_robot_ids:
            status = Status()
            robot = self._objects.get_robot_by_id(robot_id)
            if robot.get_role() == "GK":
                status.status = "keeper"
            elif robot.get_role() == "LDF":
                #固定値へ移動
                status.status = "move_linear"
                status.pid_goal_pos_x = -4.4
                status.pid_goal_pos_y = 1.2
                status.pid_goal_theta = math.atan2((self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]))
            elif robot.get_role() == "RDF":
                #固定値へ移動
                status.status = "move_linear"
                status.pid_goal_pos_x = -4.4
                status.pid_goal_pos_y = 0.4
                status.pid_goal_theta = math.atan2((self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]))
            elif robot.get_role() == "LFW":
                #固定値へ移動
                status.status = "move_linear"
                status.pid_goal_pos_x = -4.4
                status.pid_goal_pos_y = -0.4
                status.pid_goal_theta = math.atan2((self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]))
            elif robot.get_role() == "RFW":
                #固定値へ移動
                status.status = "move_linear"
                status.pid_goal_pos_x = -4.4
                status.pid_goal_pos_y = -1.2
                status.pid_goal_theta = math.atan2((self._ball_params.get_current_position()[1] - robot.get_current_position()[1]) , (self._ball_params.get_current_position()[0] - robot.get_current_position()[0]))
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)


        result = self._dynamic_strategy
        return result
