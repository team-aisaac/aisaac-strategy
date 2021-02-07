#!/usr/bin/env  python
# coding:utf-8
import rospy

from referee import Referee
from objects import Objects

import strategy
from strategy_calcurator import StrategyCalcuratorBase
from normal_start_strategy_calcurator import NormalStartStrategyCalcurator, NormalStartKickOffStrategyCalcurator, NormalStartKickOffDefenceStrategyCalcurator, NormalStartPenaltyDefenceStrategyCalcurator
from stop_strategy_calcurator import StopStrategyCalculator
from direct_free_attack_strategy_calcurator import DirectFreeAttack
from direct_free_defence_strategy_calcurator import DirectFreeDefence
from indirect_free_attack_strategy_calcurator import IndirectFreeAttack
from indirect_free_defence_strategy_calcurator import IndirectFreeDefence
from penalty_attack_strategy_calcurator import PenaltyAttack
from friend_ball_placement_strategy_calcurator import FriendBallPlacement
from enemy_ball_placement_strategy_calculator import EnemyBallPlacement
from context import StrategyContext
from world_model_status_publisher import WorldModelStatusPublisher

from std_msgs.msg import String, Float32

from filter import identity_filter
import robot
import threading
import time

import config

"""
主にフィールドの情報の取得、物理的な衝突の確認をするコード
"""


class WorldModel(object):
    def __init__(self):
        rospy.init_node("world_model")
        self._team_color = str(rospy.get_param('friend_color'))
        self._team_side = str(rospy.get_param('team_side'))

        """----上の5つの変数、インスタンスをまとめたもの、callbackをもつ---"""
        self._objects = Objects(
            self._team_color, self._team_side, config.NUM_FRIEND_ROBOT, config.NUM_ENEMY_ROBOT, node="world_model")

        self.robot_pys = []
        self.robot_py_ths = []

        self.robot_ids = self._objects.get_robot_ids()

        # スレッドを使わないときはここをコメントアウト --ここから--
        for robot_id in self.robot_ids:
            robot_py = robot.Robot(robot_id)
            self.robot_pys.append(robot_py)
            th = threading.Thread(target=robot_py.run)
            th.setDaemon(True)
            th.start()
            self.robot_py_ths.append(th)
        # スレッドを使わないときはここをコメントアウト --ここまで--

        """---Referee---"""
        self._referee = Referee(self._objects)
        self._stcalcurator = {
            'normal_start_normal': NormalStartStrategyCalcurator(self._objects),
            'normal_start_kickoff': NormalStartKickOffStrategyCalcurator(self._objects),
            'normal_start_kickoff_defence': NormalStartKickOffDefenceStrategyCalcurator(self._objects),
            'stop': StopStrategyCalculator(self._objects),
            'penalty_attack': PenaltyAttack(self._objects),
            'penalty_defence': NormalStartPenaltyDefenceStrategyCalcurator(self._objects),
            'direct_free_attack': DirectFreeAttack(self._objects),
            'direct_free_defence': DirectFreeDefence(self._objects),
            'indirect_free_attack': IndirectFreeAttack(self._objects),
            'indirect_free_defence': IndirectFreeDefence(self._objects),
            'friend_ball_placement': FriendBallPlacement(self._objects),
            'enemy_ball_placement': EnemyBallPlacement(self._objects)
        }
        self._status_publisher = WorldModelStatusPublisher(
            self._team_color, robot_ids=self._objects.get_robot_ids())

        # 積分などに必要な情報を保存するオブジェクト
        self._strategy_context = StrategyContext()

        # とりあえず例として"last_number"という名前で10フレーム分のコンテキストを作成。
        # 初期値は全て0を指定。
        self._strategy_context.register_new_context("last_number", 10, 0)
        self._strategy_context.register_new_context(
            "normal_strat_state", 2, 0, namespace="normal_strat")
        self._strategy_context.register_new_context(
            "kickoff_complete", 1, False, namespace="world_model")
        self._strategy_context.register_new_context(
            "enemy_kick", 1, False, namespace="world_model")
        self._strategy_context.register_new_context(
            "referee_branch", 1, "NONE", namespace="world_model")
        self._strategy_context.register_new_context(
            "defence_or_attack", 1, False, namespace="world_model")
        self._strategy_context.register_new_context(
            "placed_ball_position", 1, [0, 0], namespace="world_model")
        self._strategy_context.register_new_context(
            "indirect_finish", 1, False, namespace="world_model")
        self._strategy_context.register_new_context(
            "direct_finish", 1, False, namespace="world_model")
        self._strategy_context.register_new_context(
            "penalty_finish", 1, False, namespace="world_model")

        self._loop_events = []

        custom_referee_branch_subs = rospy.Subscriber("/force_referee_branch", String, self.force_referee_branch)
        self.custom_referee_branch = ""

        self.world_model_fps_publisher = rospy.Publisher("fps/world_model", Float32, queue_size=1)

    def force_referee_branch(self, msg):
        self.custom_referee_branch = msg.data

    def add_loop_event_listener(self, callback):
        self._loop_events.append(callback)

    def trigger_loop_events(self):
        for callback in self._loop_events:
            callback()

    def get_referee(self):
        # type: () -> Referee
        return self._referee

    def get_status_publisher(self):
        # type: () -> WorldModelStatusPublisher
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

    def print_fps_callback(self, event):
        if len(self._fps) > 0:
            msg = Float32()
            msg.data = sum(self._fps)/len(self._fps)
            self.world_model_fps_publisher.publish(msg)
            self._fps = []

def run_world_model():
    world_model = WorldModel()
    loop_rate = rospy.Rate(config.WORLD_LOOP_RATE)

    rospy.loginfo("setting global param")
    rospy.set_param("/robot_max_velocity", config.ROBOT_MAX_VELOCITY)

    world_model._fps = []
    world_model._current_loop_time = 0
    world_model._last_loop_time = 0
    rospy.Timer(rospy.Duration(1.0), world_model.print_fps_callback)

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

        # 1ループ前のreferee_branch
        tmp_last_referee_branch = "NONE"
        # 変更前のreferee_branch
        last_referee_branch = "NONE"

        while not rospy.is_shutdown():
            world_model._current_loop_time = time.time()
            elapsed_time = world_model._current_loop_time - world_model._last_loop_time
            world_model._fps.append(1./elapsed_time)

            # 恒等関数フィルタの適用
            # vision_positionからcurrent_positionを決定してつめる
            for robot in world_model.get_objects().robot:
                identity_filter(robot)
            for enemy in world_model.get_objects().enemy:
                identity_filter(enemy)

            referee_branch = referee.get_referee_branch()

            if world_model.custom_referee_branch != "":
                referee_branch = world_model.custom_referee_branch

            # referee_branch = "STOP"
            #referee_branch = "INDIRECT_FREE_ATTACK"

            #strat = strategy.StopStaticStrategy()

            if referee_branch == "HALT":
                strat = strategy.HaltStaticStrategy()

            elif referee_branch == "STOP":
                strat_calcrator = world_model.get_strategy_calcurator("stop")
                strat = strat_calcrator.calcurate(strat_ctx)

            elif referee_branch == "NORMAL_START":
                if not strat_ctx.get_last("kickoff_complete", namespace="world_model") \
                        and last_referee_branch == "KICKOFF_ATTACK":
                    # 前のreferee_branchがKICKOFF_ATTACKかつkickoff終了してない場合
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'normal_start_kickoff')
                    strat = strat_calcrator.calcurate(strat_ctx)
                    # 前のreferee_branchがKICKOFF_DEFENCEかつenemy_kick終了してない場合
                elif not strat_ctx.get_last("penalty_finish", namespace="world_model") \
                        and last_referee_branch == "PENALTY_ATTACK":
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'penalty_attack')
                    strat = strat_calcrator.calcurate(strat_ctx, should_wait=False)

                elif not strat_ctx.get_last("enemy_kick", namespace="world_model") \
                        and last_referee_branch == "KICKOFF_DEFENCE":
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'normal_start_kickoff_defence')
                    strat = strat_calcrator.calcurate(strat_ctx, referee_branch)
                elif not strat_ctx.get_last("enemy_kick", namespace="world_model") \
                        and last_referee_branch == "PENALTY_DEFENCE":
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'penalty_defence')
                    strat = strat_calcrator.calcurate(strat_ctx, referee_branch)
                else:
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'normal_start_normal')
                    strat = strat_calcrator.calcurate(strat_ctx)

            elif referee_branch == "FORCE_START":
                strat_ctx.update("defence_or_attack", True, namespace="world_model")
                strat_calcrator = world_model.get_strategy_calcurator(
                    'normal_start_normal')
                strat = strat_calcrator.calcurate(strat_ctx)

            elif referee_branch == "KICKOFF_ATTACK":
                strat_ctx.update("kickoff_complete", False, namespace="world_model")
                strat = strategy.InitialStaticStrategy()
            elif referee_branch == "KICKOFF_DEFENCE":
                strat_ctx.update("enemy_kick", False, namespace="world_model")
                strat_calcrator = world_model.get_strategy_calcurator(
                    'normal_start_kickoff_defence')
                strat = strat_calcrator.calcurate(strat_ctx, referee_branch)

            elif referee_branch == "PENALTY_ATTACK":
                # if not strat_ctx.get_last("penalty_finish", namespace="world_model"):
                strat_ctx.update("penalty_finish", False, namespace="world_model")
                strat_calcrator = world_model.get_strategy_calcurator(
                    'penalty_attack')
                strat = strat_calcrator.calcurate(strat_ctx, should_wait=True)
                # else:
                #     strat_calcrator = world_model.get_strategy_calcurator(
                #         'normal_start_normal')
                strat = strat_calcrator.calcurate(strat_ctx)
            elif referee_branch == "PENALTY_DEFENCE":
                strat_calcrator = world_model.get_strategy_calcurator(
                    'penalty_defence')
                strat = strat_calcrator.calcurate(strat_ctx, referee_branch)

            elif referee_branch == "DIRECT_FREE_ATTACK":
                if not strat_ctx.get_last("direct_finish", namespace="world_model"):
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'direct_free_attack')
                else:
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'normal_start_normal')
                strat = strat_calcrator.calcurate(strat_ctx)

            elif referee_branch == "DIRECT_FREE_DEFENCE":
                    # enemy_kick終了してない場合
                if not strat_ctx.get_last("enemy_kick", namespace="world_model"):
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'direct_free_defence')
                else:
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'normal_start_normal')
                strat = strat_calcrator.calcurate(strat_ctx)

            elif referee_branch == "INDIRECT_FREE_ATTACK":
                if not strat_ctx.get_last("indirect_finish", namespace="world_model"):
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'indirect_free_attack')
                else:
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'normal_start_normal')
                strat = strat_calcrator.calcurate(strat_ctx)
            elif referee_branch == "INDIRECT_FREE_DEFENCE":
                    # enemy_kick終了してない場合
                if not strat_ctx.get_last("enemy_kick", namespace="world_model"):
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'indirect_free_defence')
                else:
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'normal_start_normal')
                strat = strat_calcrator.calcurate(strat_ctx)

            elif referee_branch == "BALL_PLACEMENT_FRIEND":
                strat_ctx.update("enemy_kick", False, namespace="world_model")
                strat_calcrator = world_model.get_strategy_calcurator("friend_ball_placement")
                place_ball_position = referee.get_place_ball_position()
                strat = strat_calcrator.calcurate(strat_ctx, place_ball_position)
            elif referee_branch == "BALL_PLACEMENT_ENEMY":
                strat_ctx.update("enemy_kick", False, namespace="world_model")
                strat_calcrator = world_model.get_strategy_calcurator("enemy_ball_placement")
                strat = strat_calcrator.calcurate(strat_ctx, referee.get_place_ball_position())

            # referee_branchが変更されたときに呼び出される
            if tmp_last_referee_branch != referee_branch:
                rospy.loginfo("Referee branch changed: "+str(referee_branch))
                rospy.set_param("/robot_max_velocity", config.ROBOT_MAX_VELOCITY)
                last_referee_branch = tmp_last_referee_branch
                strat_ctx.update("referee_branch", tmp_last_referee_branch, namespace="world_model")
                if referee_branch == "INDIRECT_FREE_ATTACK":
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'indirect_free_attack').reset()
                    strat_ctx.update("indirect_finish", False, namespace="world_model")
                if referee_branch == "DIRECT_FREE_ATTACK":
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'direct_free_attack').reset()
                    strat_ctx.update("direct_finish", False, namespace="world_model")
                if referee_branch == "PENALTY_ATTACK":
                    strat_calcrator = world_model.get_strategy_calcurator(
                        'penalty_attack').reset()
                    strat_ctx.update("penalty_finish", False, namespace="world_model")



            tmp_last_referee_branch = referee_branch

            
            status_publisher.publish_all(strat)
            world_model.trigger_loop_events()

            # # robot.py kousinn
            # for robot_py in world_model.robot_pys:
            #     robot_py.run_once()
            #     #print(robot_py.robot_id)

            world_model._last_loop_time = world_model._current_loop_time
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
            if rospy.get_param("is_test", False):
                break
