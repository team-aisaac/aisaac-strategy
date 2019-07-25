#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import config
from aisaac.msg import Status
import copy


class RobotStatusPackage(object):
    msg = Status()


class StrategyBase(object):
    def __init__(self):
        self._all_robot_status = {}
        for i in range(config.NUM_ENEMY_ROBOT):
            self._all_robot_status[i] = RobotStatusPackage()

    def get_all_robot_status(self):
        return self._all_robot_status

    def get_robot_status(self, robot_id):
        return self._all_robot_status[robot_id]


class InitialStrategy(StrategyBase):
    def __init__(self):
        super(StrategyBase, self).__init__()

        status = Status()

        status.pid_goal_pos_x = 0.0
        status.pid_goal_pos_y = 0.0
        self._all_robot_status[0] = copy.deepcopy(status)

        status.pid_goal_pos_x = 1.0
        status.pid_goal_pos_y = 0.0
        self._all_robot_status[1] = copy.deepcopy(status)

        status.pid_goal_pos_x = -1.0
        status.pid_goal_pos_y = 0.0
        self._all_robot_status[2] = copy.deepcopy(status)

        status.pid_goal_pos_x = 0.0
        status.pid_goal_pos_y = -1.0
        self._all_robot_status[3] = copy.deepcopy(status)


class StrategyCalcurator(object):
    def __init__(self):
        self.result = StrategyBase()

    def calcurate(self):
        # TODO: 実装
        # if ロボットがこの位置だったら
        #    result = SomeStrategy1()
        # elif ロボットがこうだったら
        #    result = SomeStrategy2()

        result = InitialStrategy()

        return result
