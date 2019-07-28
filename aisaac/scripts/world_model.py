#!/usr/bin/env  python
# coding:utf-8
import rospy

from referee import Referee
from objects import Objects

import strategy
from strategy_calcurator import StrategyCalcurator
from world_model_status_publisher import StatusPublisher

import config

"""
主にフィールドの情報の取得、物理的な衝突の確認をするコード
"""

class WorldModel():
    def __init__(self):
        rospy.init_node("world_model")
        self._team_color = str(rospy.get_param('team_color'))

        """----上の5つの変数、インスタンスをまとめたもの、callbackをもつ---"""
        self._objects = Objects(
            self._team_color, config.NUM_FRIEND_ROBOT, config.NUM_ENEMY_ROBOT)

        """---Referee---"""
        self._referee = Referee(self._objects)
        self._stcalcurator = StrategyCalcurator(self._objects)
        self._status_publisher = StatusPublisher(self._team_color)

    def get_referee(self):
        return self._referee

    def get_status_publisher(self):
        return self._status_publisher

    def get_strategy_calcurator(self):
        return self._stcalcurator


if __name__ == "__main__":
    world_model = WorldModel()
    loop_rate = rospy.Rate(config.WORLD_LOOP_RATE)
    print("start world model node")
    # assignment_x = [-4, -3, -2, -1, 0, 1, 2, 3]
    # assignment_y = [1, 1, 1, 1, 1, 1, 1, 1]
    # assignment_theta = [0, 0, 0, 0, 0, 0, 0, 0]
    # time.sleep(10)
    # world_model.decision_maker.goal_assignment(assignment_x, assignment_y, assignment_theta)

    status_publisher = world_model.get_status_publisher()
    try:
        referee = world_model.get_referee()
        strategy_calcurator = world_model.get_strategy_calcurator()
        while not rospy.is_shutdown():
            # referee_branch = referee.get_referee_branch()
            referee_branch = "NORMAL_START"
            strat = strategy.StopStrategy()

            if referee_branch == "HALT":
                strat = strategy.HaltStrategy()
            elif referee_branch == "STOP":
                strat = strategy.StopStrategy()
            elif referee_branch == "NORMAL_START":
                strat = strategy_calcurator.calcurate()
            elif referee_branch == "KICKOFF":
                strat = strategy.KickOffStrategy()
            elif referee_branch == "DEFENCE":
                strat = strategy.DefenceStrategy()

            # world_model.decision_maker.change_goal_status()
            # status_publisher.publish_all(strat)
            loop_rate.sleep()
    except:
        import traceback
        traceback.print_exc()
        status_publisher.publish_all(strategy.StopStrategy())
        # world_model.decision_maker.stop_all()
