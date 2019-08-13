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
        self._robots_near_to_ball = None

    def _attack_strategy1(self, strategy_context=None):
        pass_positions = [
            [1.0, 2.0],
            [2.0, -2.0],
            [3.0, 2.0],
            [6.0, 0.0],
        ]
        succeeded_area = self._objects.robot[0].size_r + 0.3

        last_state = strategy_context.get_last("normal_strat_state", namespace="normal_strat")

        ball_pos = self._objects.ball.get_current_position()

        if self._receiver_id is None:
            target_pos = pass_positions[last_state]
        else:
            # パスの目標ではなく、受け取る側のロボットが実際にリーチした場所を目標とする
            target_pos = self._objects.robot[self._receiver_id].get_current_position()

        distance = functions.distance_btw_two_points(ball_pos, target_pos)
        # print("Receiver: "+ str(self._receiver_id) +" Distance:"+str(distance))

        cur_state = last_state

        # 目標位置近くにボールが行ったら次のステートへ
        change_state = False
        if distance < succeeded_area and len(pass_positions) > cur_state:
            cur_state = cur_state + 1
            change_state = True

        # 最後のpass_positionsだったらシュート
        is_shoot = False
        if len(pass_positions) - 1 == cur_state:
            is_shoot = True

        # シュートが終了したらリセット
        should_reset = False
        if len(pass_positions) == cur_state:
            cur_state = cur_state - 1
            should_reset = True

        # print("State:"+str(cur_state))

        # InitialStaticStrategyを元に組み立てる
        self._dynamic_strategy.clone_from(self._static_strategies['initial'])

        not_assigned_robot_ids = self._get_active_robot_ids()

        if len(not_assigned_robot_ids) >= 5:
            # 残り5台以上の場合
            # Defence系を先にアサイン
            not_assigned_ops = ["keeper", "defence3", "defence4"]
        elif len(not_assigned_robot_ids) == 4:
            # 残り4台の場合
            not_assigned_ops = ["keeper", "defence3"]
        elif len(not_assigned_robot_ids) == 3:
            # 残り3台の場合
            not_assigned_ops = ["keeper"]
        else:
            # 残り3台未満の場合
            not_assigned_ops = ["keeper"]

        # for文で消えないようコピー
        sorted_not_assigned_robot_ids = sorted(not_assigned_robot_ids, reverse=True)

        # Defence系をアサイン
        for robot_id in sorted_not_assigned_robot_ids:
            if not_assigned_ops == []:
                break
            status = Status()
            status.status = not_assigned_ops[0]

            self._dynamic_strategy.set_robot_status(robot_id, status)

            not_assigned_ops.pop(0)
            not_assigned_robot_ids.remove(robot_id)

        # ステートが変わるときに近い順を更新
        if self._robots_near_to_ball is None or change_state:
            self._robots_near_to_ball = self._objects.get_robot_ids_sorted_by_distance_to_ball(not_assigned_robot_ids)

        if len(self._robots_near_to_ball) >= 2: # 2台以上攻撃に割ける場合
            # 残りをボールに近い順にアサイン
            for idx, robot_id in enumerate(self._robots_near_to_ball):
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
                    status.pid_goal_pos_x = pass_positions[cur_state][0]
                    status.pid_goal_pos_y = pass_positions[cur_state][1]
                    not_assigned_robot_ids.remove(robot_id)

                self._dynamic_strategy.set_robot_status(robot_id, status)

        else: # 1台以下の場合あまりがいればとりあえずゴールにシュート
            for robot_id in self._robots_near_to_ball:
                status = Status()
                status.status = "pass"
                status.pass_target_pos_x = pass_positions[-1][0]
                status.pass_target_pos_y = pass_positions[-1][1]
                self._dynamic_strategy.set_robot_status(robot_id, status)

        # self._dynamic_strategy.clone_from(self._static_strategies['initial'])
        if should_reset:
            # シュートしたらリセット
            strategy_context.update("normal_strat_state", 0, namespace="normal_strat")
            self._receiver_id = None
        else:
            strategy_context.update("normal_strat_state", cur_state, namespace="normal_strat")

        result = self._dynamic_strategy
        return result

    """
    referee_branchがNORMAL_STARTの場合のCalcurator。
    """
    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase
        if True:
            result = self._attack_strategy1(strategy_context)

        return result


class NormalStartKickOffStrategyCalcurator(StrategyCalcuratorBase):
    def __init__(self, objects):
        super(NormalStartKickOffStrategyCalcurator, self).__init__(objects)
        self._receiver_id = None
        self._kicker_id = None

    def calcurate(self, strategy_context=None):
        pass_pos = [-0.3, -2.0]
        succeeded_area = self._objects.robot[0].size_r + 0.3

        ball_pos = self._objects.ball.get_current_position()
        target_pos = pass_pos
        distance = functions.distance_btw_two_points(ball_pos, target_pos)

        if distance < succeeded_area:
            strategy_context.update("kickoff_complete", True, namespace="world_model")

        self._dynamic_strategy.clone_from(self._static_strategies['initial'])

        not_assigned_robot_ids = self._get_active_robot_ids()

        if len(not_assigned_robot_ids) >= 5:
            # 残り5台以上の場合
            # Defence系を先にアサイン
            not_assigned_ops = ["keeper", "defence3", "defence4"]
        elif len(not_assigned_robot_ids) == 4:
            # 残り4台の場合
            not_assigned_ops = ["keeper", "defence3"]
        elif len(not_assigned_robot_ids) == 3:
            # 残り3台の場合
            not_assigned_ops = ["keeper"]
        else:
            # 残り3台未満の場合
            not_assigned_ops = ["keeper"]

        # for文で消えないようコピー
        sorted_not_assigned_robot_ids = sorted(not_assigned_robot_ids, reverse=True)

        # Defence系をアサイン
        for robot_id in sorted_not_assigned_robot_ids:
            if not_assigned_ops == []:
                break
            status = Status()
            status.status = not_assigned_ops[0]

            self._dynamic_strategy.set_robot_status(robot_id, status)

            not_assigned_ops.pop(0)
            not_assigned_robot_ids.remove(robot_id)

        robot_ids = self._objects.get_robot_ids_sorted_by_distance_to_ball(not_assigned_robot_ids)

        for idx, robot_id in enumerate(robot_ids):
            status = Status()
            if idx == 0:
                status.status = "pass"
                if self._receiver_id is not None:
                    receiver_pos = self._objects.robot[self._receiver_id].get_current_position()
                    receiver_area = 1.0
                    if functions.distance_btw_two_points(pass_pos, receiver_pos) > receiver_area:
                        status.status = "prepare_pass"

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

    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase

        active_robot_ids = self._get_active_robot_ids()
        active_enemy_ids = self._get_active_enemy_ids()
        status = Status()
        nearest_enemy_id = self._objects.get_enemy_ids_sorted_by_distance_to_ball(active_enemy_ids)[0]
        for idx, robot_id in enumerate(active_robot_ids):
            if idx == 0:
                status.status = "keeper"
            elif idx == 1:
                status.status = "defence1"
            elif idx == 2:
                status.status = "defence2"
            elif idx == 3:
                #敵kickerとballの延長線上に移動
                status.status = "move_linear"
                if nearest_enemy_id != None:
                    status.pid_goal_pos_x, status.pid_goal_pos_y = functions.calculate_internal_dividing_point(self._enemy[nearest_enemy_id].get_current_position()[0], self._enemy[nearest_enemy_id].get_current_position()[1], self._ball_params.get_current_position()[0], self._ball_params.get_current_position()[1], functions.distance_btw_two_points(self._enemy[nearest_enemy_id].get_current_position(), self._ball_params.get_current_position()) + 0.8, -0.8)
                    status.pid_goal_theta = math.atan2((self._ball_params.get_current_position()[1] - self._robot[3].get_current_position()[1]) , (self._ball_params.get_current_position()[0] - self._robot[2].get_current_position()[0]))
                    if status.pid_goal_pos_x >= -0.2 and status.pid_goal_pos_y >= 0:
                        status.pid_goal_pos_x = -0.2
                        status.pid_goal_pos_y = 0.8
                    elif status.pid_goal_pos_x >= -0.2 and status.pid_goal_pos_y < 0:
                        status.pid_goal_pos_x = -0.2
                        status.pid_goal_pos_y = -0.8
            elif idx == 4:
                #固定位置
                status.status = "move_linear"
                free_enemy_id = self._get_free_enemy_id(4, nearest_enemy_id)
                status.pid_goal_pos_x = -3.
                status.pid_goal_pos_y = 0.
                status.pid_goal_theta = math.atan2( (self._ball_params.get_current_position()[1] - self._robot[4].get_current_position()[1]) , (self._ball_params.get_current_position()[0] - self._robot[4].get_current_position()[0]) )
            else:
                status.status = "none"
            self._dynamic_strategy.set_robot_status(robot_id, status)


        result = self._dynamic_strategy
        return result
