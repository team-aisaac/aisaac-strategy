#!/usr/bin/env  python
# coding:utf-8
import rospy

from referee import Referee
from objects import Objects

import strategy
from normal_start_strategy_calcurator import NormalStartStrategyCalcurator
from direct_free_blue_strategy_calcurator import DirectFreeBlue
from indirect_free_blue_strategy_calcurator import IndirectFreeBlue
from context import StrategyContext
from world_model_status_publisher import WorldModelStatusPublisher

from filter import identity_filter

import config

"""
主にフィールドの情報の取得、物理的な衝突の確認をするコード
"""


class WorldModel(object):
    def __init__(self):
        rospy.init_node("world_model")
        self._team_color = str(rospy.get_param('~team_color'))

        """----上の5つの変数、インスタンスをまとめたもの、callbackをもつ---"""
        self._objects = Objects(
            self._team_color, config.NUM_FRIEND_ROBOT, config.NUM_ENEMY_ROBOT)

        """---Referee---"""
        self._referee = Referee(self._objects)
        self._stcalcurator = {
            'normal_start': NormalStartStrategyCalcurator(self._objects),
            'direct_free_blue': DirectFreeBlue(self._objects),
            'indirect_free_blue': IndirectFreeBlue(self._objects)
        }
        self._status_publisher = WorldModelStatusPublisher(
            self._team_color, robot_ids=self._objects.get_robot_ids())

        # 積分などに必要な情報を保存するオブジェクト
        self._strategy_context = StrategyContext()

        # とりあえず例として"last_number"という名前で10フレーム分のコンテキストを作成。
        # 初期値は全て0を指定。
        self._strategy_context.register_new_context("last_number", 10, 0)
        self._strategy_context.register_new_context("normal_strat_state", 2, 0)
        self._loop_events = []

    def add_loop_event_listener(self, callback):
        self._loop_events.append(callback)

    def trigger_loop_events(self):
        for callback in self._loop_events:
            callback()

    def get_referee(self):
        # type: () -> Referee
        return self._referee

    def get_status_publisher(self):
        # type: () -> StatusPublisher
        return self._status_publisher

    def get_strategy_calcurator(self, key):
        # type: () -> StrategyCalcuratorBase
        return self._stcalcurator[key]

    def get_strategy_context(self):
        # type: () -> StrategyContext
        return self._strategy_context

    def get_objects(self):
        # type: () -> Objects
        return self._objects


def run_world_model():
    world_model = WorldModel()
    loop_rate = rospy.Rate(config.WORLD_LOOP_RATE)
    rospy.loginfo("start world model node")
    # assignment_x = [-4, -3, -2, -1, 0, 1, 2, 3]
    # assignment_y = [1, 1, 1, 1, 1, 1, 1, 1]
    # assignment_theta = [0, 0, 0, 0, 0, 0, 0, 0]
    # time.sleep(10)
    # world_model.decision_maker.goal_assignment(
    #   assignment_x, assignment_y, assignment_theta)

    status_publisher = world_model.get_status_publisher()

    try:
        referee = world_model.get_referee()
        strat_ctx = world_model.get_strategy_context()

        world_model.add_loop_event_listener(strat_ctx.handle_loop_callback)

        while not rospy.is_shutdown():
            # 恒等関数フィルタの適用
            # vision_positionからcurrent_positionを決定してつめる
            for robot in world_model.get_objects().robot:
                identity_filter(robot)
            for enemy in world_model.get_objects().enemy:
                identity_filter(enemy)

            referee_branch = referee.get_referee_branch()
            # referee_branch = "NORMAL_START"
            #strat = strategy.StopStaticStrategy()

            if referee_branch == "HALT":
                strat = strategy.HaltStaticStrategy()
            elif referee_branch == "STOP":
                strat = strategy.StopStaticStrategy()
            elif referee_branch == "NORMAL_START":
                strat_calcrator = world_model.get_strategy_calcurator(
                    'normal_start')
                strat = strat_calcrator.calcurate(strat_ctx)
            elif referee_branch == "KICKOFF":
                strat = strategy.KickOffStaticStrategy()
            elif referee_branch == "DEFENCE":
                strat = strategy.DefenceStaticStrategy()
            elif referee_branch == "DIRECT_FREE_BLUE":
                strat_calcrator = world_model.get_strategy_calcurator(
                    'direct_free_blue')
                strat = strat_calcrator.calcurate(strat_ctx)
            elif referee_branch == "INDIRECT_FREE_BLUE":
                strat_calcrator = world_model.get_strategy_calcurator(
                    'indirect_free_blue')
                strat = strat_calcrator.calcurate(strat_ctx)

            status_publisher.publish_all(strat)
            world_model.trigger_loop_events()
            loop_rate.sleep()
    except Exception as e:
        import traceback
        traceback.print_exc()
        status_publisher.publish_all(strategy.StopStaticStrategy())
        # world_model.decision_maker.stop_all()


if __name__ == "__main__":
    while True and not rospy.is_shutdown():
        try:
            run_world_model()
        except:
            import traceback
            traceback.print_exc()
