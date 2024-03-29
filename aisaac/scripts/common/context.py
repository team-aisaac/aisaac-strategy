#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import rospy

try:
    from typing import Dict, List
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class ContextBase(object):
    """
    過去のループでの情報をFPSに同期して保持しておくためのクラス
    """

    def __init__(self):
        self._context = {}  # type: Dict[str, List[object]]
        self._new_context = {}  # type: Dict[str, object]
        self._default_value = {}  # type: Dcit[str, object]

        # dtを得るための時間情報
        self.register_new_context("__last_loop_time__", 2, rospy.Time.now())

        self._updated = False

    def register_new_context(self, key, length, default, strict=True, namespace=""):
        # type: (str, int, object, bool, str) -> None
        # key: データを一意に特定するための名前
        # length: 何フレーム分保存しておくか
        # default: 初期化時に配列を埋めるのに利用するデータ

        _key = str(namespace) + "/" + str(key)

        if not _key in self._context:
            self._context[_key] = [default for _ in range(length)]
            self._default_value[_key] = default
        else:
            if strict:
                raise Exception("もう既に登録されたコンテキストのkeyです:" + _key)
            else:
                pass

        self._last_loop_time = rospy.Time.now()

    def handle_loop_callback(self):
        # ループ一回につき１度だけ呼ぶイベント。必ずフレームと同じ回数(60FPSなら60回/秒)だけ呼ぶこと。

        self.update("__last_loop_time__", rospy.Time.now())
        self._updated = True

        for _key in self._new_context:
            self._context[_key].append(copy.deepcopy(self._new_context[_key]))
            self._context[_key].pop(0)
        self._new_context = {}

    def is_valid(self):
        # 一度もcallbackが呼ばれていない場合、このコンテキストは管理されていない。
        # handle_loop_callbackを毎ループ呼ぶ必要がある。
        return self._updated

    def update(self, key, new_value, namespace=""):
        # type: (str, object, str) -> None
        # 次のデータを登録する。実際の更新の実行は１ループ終了時に
        # fire_one_loop_eventが呼ばれたときのみに行われる。

        _key = str(namespace) + "/" + str(key)
        self._new_context[_key] = new_value

    def force_update(self, key, new_value, namespace=""):
        # type: (str, object, str) -> None
        # 1ループ1回までの制限にかからず、強制的にアップデートする。
        # これを使うなら値の管理は自己責任で。
        _key = str(namespace) + "/" + str(key)
        self._new_context[_key] = new_value
        self._context[_key].append(copy.deepcopy(self._new_context[_key]))
        self._context[_key].pop(0)

    def reset(self, key, default=None, namespace=""):
        # type: (str, object, str) -> None
        # リセットする。defaultが指定されていればそちらの値でリセットする。
        _key = str(namespace) + "/" + str(key)
        if default:
            default_value = default
        else:
            default_value = self._default_value[_key]

        length = len(self._context[_key])
        self._context[_key] = [default_value for _ in range(length)]

    def get_dt(self):
        # type: () -> float
        # ループ間でかかった時間(dt)をfloatで返す。単位は秒。
        rostimes = self.get_all("__last_loop_time__")  # type: List[rospy.Time]
        return (rostimes[-1] - rostimes[-2]).to_sec()

    def get_all(self, key, namespace=""):
        # type: (str, str) -> object
        # 保存している情報をリストで全て返す。先頭が最も古いもの。最後が最も新しいもの。
        _key = str(namespace) + "/" + str(key)
        return self._context[_key]

    def get_last(self, key, namespace=""):
        # type: (str, str) -> object
        # 一つ前のループのデータを取得できる。１ループが終了し、
        # fire_one_loop_eventが呼ばれるまで更新されない。
        _key = str(namespace) + "/" + str(key)
        return self._context[_key][-1]

    def get_oldest(self, key, namespace=""):
        # type: (str, str) -> object
        # 最も古いデータを取得する関数
        _key = str(namespace) + "/" + str(key)
        return self._context[_key][0]


class RobotContext(ContextBase):
    pass


class StrategyContext(ContextBase):
    pass
