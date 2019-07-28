#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import strategy

class StrategyCalcurator(object):
    def __init__(self, objects):
        self._objects = objects

    def calcurate(self):
        result = strategy.InitialStrategy()

        # TODO: 実装
        # if ロボットがこの位置だったら
        #    result = SomeStrategy1()
        # elif ロボットがこうだったら
        #    result = SomeStrategy2()

        return result
