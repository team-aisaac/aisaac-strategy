#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from strategy import StrategyBase, InitialStrategy
import math
from objects import Objects

try:
    from typing import Tuple
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


def get_distance(point_a, point_b):
    # type: (Tuple[float, float], Tuple[float, float]) -> float
    return math.sqrt((point_a[0] - point_b[0])**2
                     + (point_a[1] - point_b[1])**2)


class StrategyCalcurator(object):
    def __init__(self, objects):
        # type: (Objects) -> None
        self._objects = objects
        self.robot = objects.robot
        self.enemy = objects.enemy

    def _get_who_has_a_ball(self):
        # type: () -> str
        ball_position = self._objects.ball.get_current_position()
        size_r = self._objects.robot.size_r
        flag = 0
        area = 0.015

        for i in self._objects.get_robot_ids():
            robot_position = self.robot[i].get_current_position()
            if get_distance(ball_position, robot_position) \
                    < math.sqrt((size_r + area)**2):
                self.robot[i].has_a_ball = True
                # print("check:",i)
                flag = flag + 1
            else:
                self.robot[i].has_a_ball = False

        for i in self._objects.get_enemy_ids():
            enemy_position = self.enemy[i].get_current_position()

            if get_distance(ball_position, enemy_position) \
                    < math.sqrt((size_r + area)**2):
                self.enemy[i].has_a_ball = True
                flag = flag - 1
            else:
                self.enemy[i].has_a_ball = False

        if flag > 0:
            who_has_a_ball = "robots"
        elif flag < 0:
            who_has_a_ball = "enemy"
        else:
            who_has_a_ball = "free"

        return who_has_a_ball

    def calcurate(self):
        # type: () -> StrategyBase
        result = InitialStrategy()

        # TODO: 実装
        # if ロボットがこの位置だったら
        #    result = SomeStrategy1()
        # elif ロボットがこうだったら
        #    result = SomeStrategy2()

        return result
