#!/usr/bin/env python3
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
    def __init__(self, robot_ids=range(config.NUM_FRIEND_ROBOT)):
        # type: (List[int]) -> None
        self._all_robot_status = {}  # type: (Dict[int, Status])
        status = _get_default_status()

        for i in robot_ids:
            self._all_robot_status[i] = copy.deepcopy(status)

    def get_all_robot_status(self):
        # type: () -> Dict[int, Status]
        return self._all_robot_status

    def get_robot_status(self, robot_id):
        # type: (int) -> Status
        return self._all_robot_status[robot_id]


class InitialStrategy(StrategyBase):
    def __init__(self, robot_ids=range(config.NUM_FRIEND_ROBOT)):
        # type: (List[int]) -> None
        super(InitialStrategy, self).__init__(robot_ids)

        status = _get_default_status()
        status.status = 'move_linear'

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

        status.pid_goal_pos_x = 1.0
        status.pid_goal_pos_y = -1.0
        self._all_robot_status[4] = copy.deepcopy(status)

        status.pid_goal_pos_x = -1.0
        status.pid_goal_pos_y = -1.0
        self._all_robot_status[5] = copy.deepcopy(status)

        status.pid_goal_pos_x = 0.0
        status.pid_goal_pos_y = -2.0
        self._all_robot_status[6] = copy.deepcopy(status)

        status.pid_goal_pos_x = 1.0
        status.pid_goal_pos_y = -2.0
        self._all_robot_status[7] = copy.deepcopy(status)


class StopStrategy(StrategyBase):
    def __init__(self, robot_ids=range(config.NUM_FRIEND_ROBOT)):
        # type: (List[int]) -> None
        super(StopStrategy, self).__init__(robot_ids)

        status = _get_default_status()
        status.status = 'stop'

        for robot_id in self._all_robot_status.keys():
            self._all_robot_status[robot_id] = copy.deepcopy(status)
