#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from abc import ABCMeta, abstractmethod

from strategy import StrategyBase, InitialStaticStrategy, StopStaticStrategy, DynamicStrategy
from strategy_context import StrategyContext
from objects import Objects
from aisaac.msg import Status
import copy

try:
    from typing import Tuple, List, Dict
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class Util(object):
    """
    答え = 関数(引数、引数)
    の形で書ける、かつ、いろんな場所で使われるものはここに書く(書きたい)。
    """
    @staticmethod
    def get_distance(point_a, point_b):
        # type: (Tuple[float, float],
        #        Tuple[float, float]) -> float
        return math.sqrt((point_a[0] - point_b[0])**2
                         + (point_a[1] - point_b[1])**2)


class StrategyCalcuratorBase(object):
    """
    StrategyCalcuratorは計算が必要なStrategy(StaticStrategyじゃないStrategy)
    を計算するためのクラス。

    全てのStrategyCalcuratorはこのクラスを継承する（したい）。

    StrategyCalcurator内で共通で利用するような関数はこのクラスに実装する（したい）。

    共通処理のうちでも、引数だけで計算出来、状態がかかわらない計算
    （例：距離の計算は点A(x,y),点B(x,y)がわかればできる）はこのクラスではなく、
    Utilクラスに@staticmethodをつけて実装する（したい）。

    変数命名規則:
        "self._"で始まるもの：StrategyCalcuratorBaseとその子クラスのみからアクセスする変数・関数（protected）
        "self"つきで"_"無しで始まるもの：StrategyCalcurator外からもアクセスする変数・関数（public）
        ※ローカル変数は"_"無しでも良い

    ※オブジェクト指向の一般的なコツとして、クラス名はモノ、すなわち名詞にする。フィールド変数はクラスが所持している所有物やデータというイメージ。
    　フィールド変数も名詞。基本的にフィールド変数はprivateまたはprotected（"self._"つき）にして、できる限り読み取りアクセスに限定して
    　（setterはなるべく作らない方が良い）、変更が必要な場合はそのフィールド変数を持つクラスの関数が管理する。
    　（現実で言えば、自分のモノを、他人が誰でもすり替えたりできたらやばい）

    　関数名は動詞にする。主語はその関数を持っているオブジェクトか、オブジェクトを使う側になる。特にget/setは完全に使う側目線。

    　フィールドはグローバル変数みたいなものなので、増えるほど状態が管理しきれなくなっていくため、
    　同じまとまりのモノをクラスにしてしまったりするのが吉。
    　あと、オブジェクト指向に限らずネストが深いのもやばいので、ネスト深くなったら関数に出来ないか考える。
    """

    __metaclass__ = ABCMeta

    def __init__(self, objects):
        # type: (Objects) -> None
        self._objects = objects
        self._robot = self._objects.robot
        self._enemy = self._objects.enemy
        self._robot_ids = self._objects.get_robot_ids()
        self._enemy_ids = self._objects.get_enemy_ids()

        # 毎ループnewするのは重いので辞書に格納しておく
        self._static_strategies = {
            'initial': InitialStaticStrategy(),
            'stop': StopStaticStrategy()
        }

        self._dynamic_strategy = DynamicStrategy()

    def _get_active_robot_ids(self):
        # type: () -> List[int]
        active_robot_ids = self._robot_ids
        # TODO: アクティブな味方ロボットのIDリストにする
        return active_robot_ids

    def _get_active_enemy_ids(self):
        # type: () -> List[int]
        active_enemy_ids = self._enemy_ids
        # TODO: アクティブな敵ロボットのIDリストにする
        return active_enemy_ids

    def _get_who_has_a_ball(self):
        # type: () -> str
        ball_position = self._objects.ball.get_current_position()
        size_r = self._objects.robot.size_r
        flag = 0
        area = 0.015

        for i in self._objects.get_robot_ids():
            robot_position = self._robot[i].get_current_position()
            if Util.get_distance(ball_position, robot_position) \
                    < math.sqrt((size_r + area)**2):
                self._robot[i].has_a_ball = True
                # print("check:",i)
                flag = flag + 1
            else:
                self._robot[i].has_a_ball = False

        for i in self._objects.get_enemy_ids():
            enemy_position = self._enemy[i].get_current_position()

            if Util.get_distance(ball_position, enemy_position) \
                    < math.sqrt((size_r + area)**2):
                self._enemy[i].has_a_ball = True
                flag = flag - 1
            else:
                self._enemy[i].has_a_ball = False

        if flag > 0:
            who_has_a_ball = "robots"
        elif flag < 0:
            who_has_a_ball = "enemy"
        else:
            who_has_a_ball = "free"

        return who_has_a_ball

    @abstractmethod
    def calcurate(self, strategy_context=None):
        # 継承先の関数で実装
        pass


class NormalStartStrategyCalcurator(StrategyCalcuratorBase):
    """
    referee_branchがNORMAL_STARTの場合のCalcurator。
    """
    def calcurate(self, strategy_context=None):
        # type: (StrategyContext) -> StrategyBase


        # コンテキストに保存されてる情報の拾い方(last_numberはworld_modelのコンストラクタでregisterしてある)
        last_number = strategy_context.get_last("last_number")
        stored_numbers = strategy_context.get_all("last_number")
        
        # InitialStaticStrategyを元に組み立てる
        self._dynamic_strategy.clone_from(self._static_strategies['initial'])
        
        # TODO: 色々計算

        # 生きてるロボットにだけ新たな指示を出す例
        # active_robot_ids = self._get_active_robot_ids()
        # status = Status()
        # for idx, robot_id in enumerate(active_robot_ids):
        #     if idx == 1:
        #         status.status = "defence1"
        #     elif idx == 2:
        #         status.status = "defence2"
        #     else:
        #         status.status = "move_linear"
        #     self._dynamic_strategy.set_robot_status(robot_id, status)

        result = self._dynamic_strategy

        # コンテキストに保存されてる情報の更新例
        strategy_context.update("last_number", last_number+1)
        # updateを2回以上1回のループの中で呼ぶと最後に呼ばれたものが保存される
        strategy_context.update("last_number", last_number+2)

        return result
