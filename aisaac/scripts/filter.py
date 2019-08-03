#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

class KalmanFilteredPosition(object):
    '''
    位置推定のカルマンフィルタ
    精度が上がるかはわからんが、Visoion情報と速度情報を利用して
    Vision情報のノイズを下げるフィルタ
    ただし、速度情報の遅延が考えられるのでそこは考慮する必要がある
    シミュレータ上を想定して遅延なしをまず実装する
    initで位置ノイズ、観測ノイズ、初期ノイズを指定
    ノイズ行列は無相関で全部同じ値を仮定
    ---------
    [input]
    visionのtにおける(x,y,theta)の平均と分散
    robotがもつt-1における(x,y,thta)の平均と分散
    ----------
    [output]
    robotがもつtにおける(x,y,thta)
    '''
    def __init__(self, robot):
        self.robot = robot
        self.position_noise = 0.01
        self.observation_noise = 0.01
        self.initial_noise = 0.01
        self.delta_t = 1/60

    def update(self, position_before, velocity, observe_position):
        mu_prediction = position_before + self.delta_t * observe_position
        sigma_prediction = sigma_before +


if __name__ == '__main__':
    pass
