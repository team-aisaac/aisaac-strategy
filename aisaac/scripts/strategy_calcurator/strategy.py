#!/usr/bin/env python
# -*- coding: utf-8 -*-

import config
from aisaac.msg import Status
import copy

try:
    from typing import Dict, List
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


def _get_default_status():
    # type: () -> Status
    status = Status()
    status.status = "None"
    status.pid_goal_pos_x = 0.
    status.pid_goal_pos_y = 0.
    status.pid_goal_theta = 0.
    status.pid_circle_center_x = 0.
    status.pid_circle_center_y = 0.
    status.pass_target_pos_x = 0.
    status.pass_target_pos_y = 0.

    return status


class StrategyBase(object):
    """
    味方のロボットの戦略をまとめたクラス。
    このクラスをworld_model_status_publisherに渡して
    全部のロボットのプロセスに指示を出す。
    """

    def __init__(self, robot_ids=range(config.NUM_FRIEND_ROBOT), clone_base=None):
        # type: (List[int], StrategyBase) -> None

        if not clone_base:
            self._all_robot_status = {}  # type: (Dict[int, Status])
            status = _get_default_status()

            for i in robot_ids:
                self._all_robot_status[i] = copy.deepcopy(status)
        else:
            # clone_baseが指定された場合はまるごとコピーする
            self.clone_from(clone_base)

    def get_all_robot_status(self):
        # type: () -> Dict[int, Status]
        return self._all_robot_status

    def get_robot_status(self, robot_id):
        # type: (int) -> Status
        return self._all_robot_status[robot_id]

    def clone_from(self, clone_base):
        # まるごとコピー
        self._all_robot_status = copy.deepcopy(
            clone_base.get_all_robot_status())
        return self


class DynamicStrategy(StrategyBase):
    """
    戦況に応じてStatusを変えて行く必要がある場合はこのDynamicStrategyを利用
    """

    def set_robot_status(self, robot_id, status):
        # type: (int, Status) -> None
        self._all_robot_status[robot_id] = copy.deepcopy(status)


class StaticStrategy(StrategyBase):
    """
    ゲームスタート直前は配置につく、とか、審判がこの指示のときは全員停止、とか
    計算する必要の無いような場合はStaticStrategyとして定義
    """
    pass


class InitialStaticStrategy(StaticStrategy):
    """
    とりあえず適当な配置につくStrategy
    """
    def __init__(self, robot_ids=range(config.NUM_FRIEND_ROBOT)):
        # type: (List[int]) -> None
        super(InitialStaticStrategy, self).__init__(robot_ids)

        status = _get_default_status()
        status.status = 'move_linear'

        initial_positions = [
            [-4.0, 0.0],
            [-0.6, 0.0],
            [-0.6, -1.0],
            [-3.0, 1.0],
            [-3.0, -1.0]
        ]

        for robot_id in self._all_robot_status.keys():
            status.pid_goal_pos_x = initial_positions[robot_id][0]
            status.pid_goal_pos_y = initial_positions[robot_id][1]
            self._all_robot_status[robot_id] = copy.deepcopy(status)


class StopStaticStrategy(StaticStrategy):
    """
    とりあえず全機にstop命令をかけるStrategy
    """

    def __init__(self, robot_ids=range(config.NUM_FRIEND_ROBOT)):
        # type: (List[int]) -> None
        super(StopStaticStrategy, self).__init__(robot_ids)

        status = _get_default_status()
        status.status = 'stop'

        for robot_id in self._all_robot_status.keys():
            self._all_robot_status[robot_id] = copy.deepcopy(status)

class HaltStaticStrategy(StaticStrategy):
    """
    HALT指示に対するStrategy
    """

    def __init__(self, robot_ids=range(config.NUM_FRIEND_ROBOT)):
        # type: (List[int]) -> None
        super(HaltStaticStrategy, self).__init__(robot_ids)

        status = _get_default_status()
        status.status = 'halt'

        for robot_id in self._all_robot_status.keys():
            self._all_robot_status[robot_id] = copy.deepcopy(status)
