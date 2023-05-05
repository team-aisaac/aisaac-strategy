#!/usr/bin/env python
# -*- coding: utf-8 -*-

from .strategy_calcurator import StrategyCalcuratorBase
from common import functions
from aisaac.msg import Status
import numpy as np
import rospy


class StopStrategyCalculator(StrategyCalcuratorBase):
    def __init__(self, objects):
        super(StopStrategyCalculator, self).__init__(objects)

    def calcurate(self, strategy_context=None, objects=None):
        # (context.StrategyContext) -> strategy.StrategyBase
        strategy_context.update("placed_ball_position", self._ball_params.get_current_position(), namespace="world_model")
        strategy_context.update("enemy_kick", False, namespace="world_model")
        self._dynamic_strategy.clone_from(self._static_strategies['initial'])

        if rospy.get_param("/robot_max_velocity") > 1.5:
            rospy.set_param("/robot_max_velocity", 1.5)

        ball = self._objects.ball

        for robot_id in self._get_active_robot_ids():
            robot = self._objects.get_robot_by_id(robot_id)

            if robot is None:
                continue

            status = self._dynamic_strategy.get_robot_status(robot_id)
            target_pos_xy = (status.pid_goal_pos_x, status.pid_goal_pos_y)

            target_distance = 0.5 + self._objects.robot[0].size_r
            distance_ball_robot = functions.distance_btw_two_points(robot.get_current_position(), ball.get_current_position())
            distance_ball_target_pos = functions.distance_btw_two_points(target_pos_xy, ball.get_current_position())

            # 閾値を超える/超えないで振動を防ぐためのoffset
            offset = 0.05
            if distance_ball_robot < (target_distance + offset) \
                    and distance_ball_target_pos < (target_distance + offset):
                status = Status()
                status.status = "move_linear"
                vector = np.array(target_pos_xy) - np.array(ball.get_current_position())
                vector = (target_distance / np.linalg.norm(vector)) * vector
                vector = np.array(ball.get_current_position()) + vector
                status.pid_goal_pos_x = vector[0]
                status.pid_goal_pos_y = vector[1]
                self._dynamic_strategy.set_robot_status(robot_id, status)

        return self._dynamic_strategy
