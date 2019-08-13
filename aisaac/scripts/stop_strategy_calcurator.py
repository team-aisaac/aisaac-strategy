#!/usr/bin/env python
# -*- coding: utf-8 -*-

from strategy import StrategyBase, StopStaticStrategy, DynamicStrategy
from strategy_calcurator import StrategyCalcuratorBase
import functions
from aisaac.msg import Status
import copy
import numpy as np
import rospy

class StopStrategyCalculator(StrategyCalcuratorBase):
    def __init__(self, objects):
        super(StopStrategyCalculator, self).__init__(objects)

    def calcurate(self, strategy_context=None):
        # (context.StrategyContext) -> strategy.StrategyBase
        self._dynamic_strategy.clone_from(self._static_strategies['initial'])

        rospy.set_param("/robot_max_velocity", 1.5)

        ball = self._objects.ball

        for robot_id in self._get_active_robot_ids():
            robot = self._objects.robot[robot_id]

            target_distance = 0.5 + self._objects.robot[0].size_r
            distance = functions.distance_btw_two_points(robot.get_current_position(), ball.get_current_position())

            # 閾値を超える/超えないで振動を防ぐためのoffset
            offset = 0.2
            if distance <= (target_distance + offset): 
                status = Status()
                status.status = "move_linear"
                vector = np.array(robot.get_current_position()) - np.array(ball.get_current_position())
                vector = (target_distance / np.linalg.norm(vector)) * vector
                vector = np.array(ball.get_current_position()) + vector
                status.pid_goal_pos_x = vector[0]
                status.pid_goal_pos_y = vector[1]
                self._dynamic_strategy.set_robot_status(robot_id, status)

        return self._dynamic_strategy