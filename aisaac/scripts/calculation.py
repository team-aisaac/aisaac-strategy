#!/usr/bin/env  python
# coding:utf-8
import math
import rospy
import numpy as np
import sys
import time
from objects import Objects
from nav_msgs.msg import Odometry
from aisaac.msg import Ball_sub_params, Def_pos
import tf
import time
from statistics import mean, median,variance,stdev
import config
import functions

WORLD_LOOP_RATE = config.WORLD_LOOP_RATE

"""
主に共通した計算処理などを担当する
"""
# Publisher用クラス
class Publisher():
    def __init__(self):
        self.team_color = str(rospy.get_param("friend_color"))

        self.ball_sub_params_pub = rospy.Publisher("/" + self.team_color + "/ball_sub_params", Ball_sub_params, queue_size=10)
        self.def_pos_pub = rospy.Publisher("/" + self.team_color + "/def_pos", Def_pos, queue_size=10)

    def ball_params_publisher(self, msg):
        self.ball_sub_params_pub.publish(msg)

    def def_pos_publisher(self, msg):
        self.def_pos_pub.publish(msg)

class Calculation():
    def __init__(self):
        rospy.init_node("Calculation_node")
        
        self.robot_color = str(rospy.get_param("friend_color"))
        self.robot_side = str(rospy.get_param("team_side"))

        # Composition
        self.objects = Objects(
            self.robot_color, self.robot_side, config.NUM_FRIEND_ROBOT, config.NUM_ENEMY_ROBOT, node="calculation")

        self.robot_friend = self.objects.robot
        self.robot_enemy = self.objects.enemy

        self.ball_params = self.objects.ball

        self.ball_sub_params = Ball_sub_params()
        self.def_pos = Def_pos()

        self.ball_frame = 10    # ボールの軌道直線フィッティングと速度の計算フレーム数
        self.ball_move_threshold = 0.05 # ボールが移動したと判定する閾値[m]
        self.same_pos_count = 0 # 停止判定用カウント
        self.ball_pos_count = 0 # 計算用カウント、フレーム単位でカウント
        self.calc_flag = False  # 計算フラグ、停止判定時は計算しない
        self.ball_pos_x_array = np.array([0.0]*self.ball_frame) # ボールのx座標保存用配列
        self.ball_pos_y_array = np.array([0.0]*self.ball_frame) # ボールのy座標保存用配列
        self.ball_vel_array = np.array([0.0]*self.ball_frame)   # ボールの速度保存用配列
        self.ball_vel_x_array = np.array([0.0]*self.ball_frame) # ボールのx方向の速度保存用配列
        self.ball_vel_y_array = np.array([0.0]*self.ball_frame) # ボールのy方向の速度保存用配列
        self.ball_vel_time_array = np.array([0.0]*self.ball_frame) # 加速度計算用、時間配列

        self.ball_vel = 0.      # ボール速度
        self.ball_vel_a = 0.    # ボール速度の傾き
        self.ball_vel_b = 0.    # ボール速度の切片
        self.ball_vel_x_a = 0.  # x方向の速度の傾き
        self.ball_vel_x_b = 0.  # x方向の速度の切片
        self.ball_vel_y_a = 0.  # y方向の速度の傾き
        self.ball_vel_y_b = 0.  # y方向の速度の切片
        self.ball_stop_time_x = 0. # x方向の停止までの時間
        self.ball_stop_time_y = 0. # y方向の停止までの時間

        # 守備の時のロボットのポジション座標計算用変数
        # 現状、青チームのみ対応
        self.g_up_x = -6.0        # ゴールポストの上側のx座標:y_GL
        self.g_up_y = 0.6        # ゴールポストの上側のy座標:x_GL
        self.g_down_x = -6.0      # ゴールポストの下側のx座標:y_GR
        self.g_down_y = -0.6      # ゴールポストの下側のy座標:x_GR
        self.g_center_x = -6.0    # ゴールの中央のx座標:y_GC
        self.g_center_y = 0.0    # ゴールの中央のy座標:x_GC
        self.p_area_up_x = -4.8   # ペナルティエリアの上側の角のx座標:y_PL
        self.p_area_up_y = 1.2   # ペナルティエリアの上側の角のy座標:x_PL
        self.p_area_down_x = -4.8   # ペナルティエリアの下側の角のx座標:y_PR
        self.p_area_down_y = -1.2   # ペナルティエリアの下側の角のy座標:x_PR
        
        self.line_up_x = 0.0        # ボールとゴールポストを結んだ線と防御ラインとの交点の上側のx座標:y_LL
        self.line_up_y = 0.0        # ボールとゴールポストを結んだ線と防御ラインとの交点の上側のy座標:x_LL
        self.line_down_x = 0.0      # ボールとゴールポストを結んだ線と防御ラインとの交点の下側のx座標:y_LR
        self.line_down_y = 0.0      # ボールとゴールポストを結んだ線と防御ラインとの交点の下側のy座標:x_LR

        self.line_up_r_x = 0.0      # ロボットの半径を考慮した補正後の座標:y_LL'
        self.line_up_r_y = 0.0      # ロボットの半径を考慮した補正後の座標:x_LL'
        self.line_down_r_x = 0.0    # ロボットの半径を考慮した補正後の座標:y_LR'
        self.line_down_r_y = 0.0    # ロボットの半径を考慮した補正後の座標:x_LR'

        self.offset_r = 0.          # オフセット値
        self.robot_r = 90.0/1000.0  # ロボット半径
        self.robot_a = 1.0          # ロボットの加速度
        self.ball_MAX_SPEED = 6.5   # ボールの最大速度
        self.delay_time_ms = 100.0  # 遅延時間[ms]

        self.L_a = 0.0              # 壁のラインとボールまでの距離
        self.L_G = 0.0              # ボール到達までに移動可能な距離

    # x,yの配列とデータ数を指定して、最小二乗法を行い、傾きと切片を返す
    def reg1dim(self, x, y, n):
        # データをクリップ
        x = np.clip(x,-6.5,6.5)
        y = np.clip(y,-5.5,5.5)
        # 傾きと切片を計算
        a = np.clip(((np.dot(x, y) - y.sum()*x.sum()/n) / ((x**2.).sum() - x.sum()**2./n)),-1.0e+3,1.0e+3)
        b = np.clip((y.sum() - a * x.sum())/n,-1.0e+3,1.0e+3)
        return a, b

    # nフレーム分のボールの位置から最小二乗法を用いて傾きと切片を計算
    # 分散が1より大きかったり、ボールが止まっているとリセット
    def calc_ball_line(self):
        #直近nフレームの座標を取得
        if self.ball_pos_count < self.ball_frame:
            self.ball_pos_x_array[self.ball_pos_count] = self.ball_params.get_current_position()[0]
            self.ball_pos_y_array[self.ball_pos_count] = self.ball_params.get_current_position()[1]
            # self.ball_vel_x_array[self.ball_pos_count] = self.ball_params.get_current_velosity()[0]
            # self.ball_vel_y_array[self.ball_pos_count] = self.ball_params.get_current_velosity()[1]
            # self.ball_vel_array[self.ball_pos_count] = math.sqrt(self.ball_params.get_current_velosity()[0]**2 + self.ball_params.get_current_velosity()[1]**2)
            # self.ball_vel_time_array[self.ball_pos_count] = 1./WORLD_LOOP_RATE * self.ball_pos_count
            # 二回目以降に、前回との偏差を計算し、一定値以下なら動いてない判定とし、カウントを増やす。nフレームの半分までカウントされたら計算フラグをFalseにして
            if self.ball_pos_count > 0:
                if functions.distance_btw_two_points(
                    (self.ball_pos_x_array[self.ball_pos_count-1],self.ball_pos_y_array[self.ball_pos_count-1]),
                    (self.ball_pos_x_array[self.ball_pos_count],self.ball_pos_y_array[self.ball_pos_count])) < self.ball_move_threshold:

                    self.same_pos_count+=1
                    if self.same_pos_count >= self.ball_frame/2:
                        self.same_pos_count = self.ball_frame/2
                        self.ball_pos_count = -1
                        self.calc_flag = False
                else:
                    self.same_pos_count = 0
                    self.calc_flag = True
            self.ball_pos_count+=1
        else:
            self.ball_pos_x_array = np.roll(self.ball_pos_x_array,-1)
            self.ball_pos_y_array = np.roll(self.ball_pos_y_array,-1)
            # self.ball_vel_x_array = np.roll(self.ball_vel_x_array,-1)
            # self.ball_vel_y_array = np.roll(self.ball_vel_y_array,-1)
            # self.ball_vel_array = np.roll(self.ball_vel_array,-1)
            self.ball_pos_x_array[self.ball_pos_count-1] = self.ball_params.get_current_position()[0]
            self.ball_pos_y_array[self.ball_pos_count-1] = self.ball_params.get_current_position()[1]
            # self.ball_vel_x_array[self.ball_pos_count-1] = self.ball_params.get_current_velosity()[0]
            # self.ball_vel_y_array[self.ball_pos_count-1] = self.ball_params.get_current_velosity()[1]
            # self.ball_vel_array[self.ball_pos_count] = math.sqrt(self.ball_params.get_current_velosity()[0]**2 + self.ball_params.get_current_velosity()[1]**2)
            if functions.distance_btw_two_points(
                (self.ball_pos_x_array[self.ball_pos_count-1],self.ball_pos_y_array[self.ball_pos_count-1]),
                (self.ball_pos_x_array[self.ball_pos_count],self.ball_pos_y_array[self.ball_pos_count])) < self.ball_move_threshold:

                self.same_pos_count+=1
                if self.same_pos_count >= self.ball_frame/2:
                    self.ball_pos_count = 0
                    self.calc_flag = False
            else:
                self.same_pos_count = 0
                self.calc_flag = True
            #x,y座標の分散を計算
            x_variance = variance(self.ball_pos_x_array)
            y_variance = variance(self.ball_pos_y_array)
            #print(x_variance,y_variance)
            #分散が1より大きかったらカウントリセット
            if (x_variance > 1 or y_variance > 1):
                self.ball_pos_count = 0
                self.same_pos_count = 0
                for i in range(0,self.ball_frame):
                    self.ball_pos_x_array[i] = 0
                    self.ball_pos_y_array[i] = 0

        #print(self.ball_pos_count,self.same_pos_count)

        if self.calc_flag == True:
            a, b = self.reg1dim(self.ball_pos_x_array, self.ball_pos_y_array, self.ball_pos_count)
            self.ball_params.set_line_a(a)
            self.ball_params.set_line_b(b)
            """ #self.ball_vel_x_a, self.ball_vel_x_b = self.reg1dim(self.ball_vel_x_array, self.ball_vel_time_array, self.ball_pos_count)
            #self.ball_vel_y_a, self.ball_vel_y_b = self.reg1dim(self.ball_vel_y_array, self.ball_vel_time_array, self.ball_pos_count)
            #self.ball_vel_a, self.ball_vel_b = self.reg1dim(self.ball_vel_array, self.ball_vel_time_array, self.ball_pos_count)
            #self.ball_params.ball_sub_params.a, self.ball_params.ball_sub_params.b = self.reg1dim(self.ball_vel_x_array, self.ball_vel_time_array, self.ball_pos_count)
            # self.ball_params.ball_sub_params.future_x = 
            # self.ball_params.ball_sub_params.future_y
            #rospy.loginfo("vel_x_a:%f\tvel_x_b:%f",self.ball_vel_x_a, self.ball_vel_x_b)

            #ボールの予想停止位置を計算
            #x,y方向の現在の速度を最小二乗法で求めた直線から計算→式が違う、速度推定が必要
            #ball_fit_vel_x = self.ball_vel_x_a*self.ball_vel_time_array[self.ball_pos_count-1] + self.ball_vel_x_b
            #ball_fit_vel_y = self.ball_vel_y_a*self.ball_vel_time_array[self.ball_pos_count-1] + self.ball_vel_y_b
            #とりあえず現在速度を使う
            #ball_fit_vel_x = self.ball_params.get_current_velosity()[0]
            #ball_fit_vel_y = self.ball_params.get_current_velosity()[1]
            #停止するまでの時間を現在の速度と傾きから計算
            if self.ball_vel_x_a != 0 and self.ball_vel_y_a != 0:
                self.ball_stop_time_x = -(ball_fit_vel_x / self.ball_vel_x_a)
                self.ball_stop_time_y = -(ball_fit_vel_y / self.ball_vel_y_a)
                if self.ball_stop_time_x <= 0 or self.ball_stop_time_y <= 0:
                    # self.ball_params.ball_sub_params.future_x = 0
                    # self.ball_params.ball_sub_params.future_y = 0
                else:
                    self.ball_params.ball_sub_params.future_x = self.ball_params.get_current_position()[0] + ball_fit_vel_x*self.ball_stop_time_x + 1/2*self.ball_vel_x_a*self.ball_stop_time_x**2
                    self.ball_params.ball_sub_params.future_y = self.ball_params.get_current_position()[1] + ball_fit_vel_y*self.ball_stop_time_y + 1/2*self.ball_vel_y_a*self.ball_stop_time_y**2
                    self.ball_params.ball_sub_params.future_x = np.clip(self.ball_params.ball_sub_params.future_x,-5,5)
                    self.ball_params.ball_sub_params.future_y = np.clip(self.ball_params.ball_sub_params.future_y,-5,5)
                    #rospy.loginfo("t=(%.3f,%.3f)\t(f_x:n_x)=(%.3f:%.3f)\t(f_y:n_y)=(%.3f:%.3f)",self.ball_stop_time_x,self.ball_stop_time_y,self.ball_params.ball_sub_params.future_x, self.ball_params.get_current_position()[0], self.ball_params.ball_sub_params.future_y, self.ball_params.get_current_position()[1]) """
        else:
            # self.ball_params.ball_sub_params.a = 0.
            # self.ball_params.ball_sub_params.b = 0.
            self.ball_params.set_line_a(0.)
            self.ball_params.set_line_b(0.)
            
            """ self.ball_vel_x_a = 0.
            self.ball_vel_x_b = 0.
            self.ball_vel_y_a = 0.
            self.ball_vel_y_b = 0.

            for i in range(0,self.ball_frame):
                self.ball_pos_x_array[i] = 0
                self.ball_pos_y_array[i] = 0 
                self.ball_vel_x_array[i] = 0
                self.ball_vel_y_array[i] = 0 """
        
        self.ball_sub_params.a = self.ball_params.get_line_a()
        self.ball_sub_params.b = self.ball_params.get_line_b()
        
        #print(self.ball_stop_time_x,self.ball_stop_time_y)
        #rospy.loginfo("f=%d\tt=(%.2f,%.2f)\t(f_x:n_x)=(%.3f:%.3f)\t(f_y:n_y)=(%.3f:%.3f)",self.calc_flag,self.ball_stop_time_x,self.ball_stop_time_y,self.ball_params.ball_sub_params.future_x, self.ball_params.get_current_position()[0], self.ball_params.ball_sub_params.future_y, self.ball_params.get_current_position()[1])

    def calc_def_pos(self):
        # 見づらいのでボールの座標を再代入
        ball_x = self.ball_params.get_current_position()[0] # y_B
        ball_y = self.ball_params.get_current_position()[1] # x_B
        # 壁の座標
        def1_pos_x = 0.0
        def1_pos_y = 0.0
        def2_pos_x = 0.0
        def2_pos_y = 0.0
        # 各パラメータ計算
        a_1 = ball_y - self.g_center_y
        b_1 = ball_x - self.g_center_x
        c_1 = self.line_down_y*(self.g_center_y - ball_y) + self.line_down_x*(self.g_center_x - ball_x)
        a_2 = ball_y - self.g_center_y
        b_2 = ball_x - self.g_center_x
        c_2 = self.line_up_y*(self.g_center_y - ball_y) + self.line_up_x*(self.g_center_x - ball_x)
        a_3 = self.g_center_y - ball_y
        b_3 = self.g_center_x - ball_x
        c_3 = self.p_area_down_y*(ball_y - self.g_center_y) + self.p_area_down_x*(ball_x - self.g_center_x)
        a_4 = ball_x - self.g_up_x
        b_4 = self.g_up_y - ball_y
        c_4 = ball_y*(self.g_up_x - ball_x) + ball_x*(ball_y - self.g_up_y)
        a_5 = ball_x - self.g_down_x
        b_5 = self.g_down_y - ball_y
        c_5 = ball_y*(self.g_down_x - ball_x) + ball_x*(ball_y - self.g_down_y)
        a_6 = self.g_center_y - ball_y
        b_6 = self.g_center_x - ball_x
        c_6 = self.p_area_up_y*(ball_y - self.g_center_y) + self.p_area_up_x*(ball_x - self.g_center_x)

        t = self.offset_r/math.sqrt((self.g_center_y - ball_y)**2 + (self.g_center_x - ball_x)**2)
        # 防御ラインの計算
        # 最下部
        if ball_x <= (self.g_down_x - self.p_area_down_x)/(self.g_down_y - self.p_area_down_y)*(ball_y - self.g_down_y) + self.g_down_x:
            self.line_up_r_y = (b_3*c_4 - b_4*c_3)/(a_3*b_4 - a_4*b_3) + (ball_y - self.g_center_y)*t
            self.line_up_r_x = (a_3*c_4 - a_4*c_3)/(a_4*b_3 - a_3*b_4) + (ball_x - self.g_center_x)*t
            self.line_down_r_y = (b_3*c_5 - b_5*c_3)/(a_3*b_5 - a_5*b_3) + (ball_y - self.g_center_y)*t
            self.line_down_r_x = (a_3*c_5 - a_5*c_3)/(a_5*b_3 - a_3*b_5) + (ball_x - self.g_center_x)*t
            self.L_a = abs(a_3*ball_y + b_3*ball_x + c_3)/math.sqrt(a_3**2 + b_3**2)

        # 下部
        elif (ball_x >= (self.g_down_x - self.p_area_down_x)/(self.g_down_y - self.p_area_down_y)*(ball_y - self.g_down_y) + self.g_down_x) and (ball_y <= self.g_center_y):
            self.line_down_r_y = (self.g_down_y - ball_y)/(self.g_down_x - ball_x)*(self.p_area_down_x - ball_x) + ball_y + (ball_y - self.g_center_y)*t
            self.line_down_r_x = self.p_area_down_x + (ball_x - self.g_center_x)*t
            self.line_down_y = (self.g_down_y - ball_y)/(self.g_down_x - ball_x)*(self.p_area_down_x - ball_x) + ball_y
            self.line_down_x = self.p_area_down_x
            c_1 = self.line_down_y*(self.g_center_y - ball_y) + self.line_down_x*(self.g_center_x - ball_x)
            self.line_up_r_y = (b_1*c_4 - b_4*c_1)/(a_1*b_4 - a_4*b_1) + (ball_y - self.g_center_y)*t
            self.line_up_r_x = (a_1*c_4 - a_4*c_1)/(a_4*b_1 - a_1*b_4) + (ball_x - self.g_center_x)*t
            self.L_a = abs(a_1*ball_y + b_1*ball_x + c_1)/math.sqrt(a_1**2 + b_1**2)

        # 上部
        elif (ball_x >= (self.g_up_x - self.p_area_up_x)/(self.g_up_y - self.p_area_up_y)*(ball_y - self.g_up_y) + self.g_up_x) and (ball_y > self.g_center_y):
            self.line_up_r_y = (self.g_up_y - ball_y)/(self.g_up_x - ball_x)*(self.p_area_up_x - ball_x) + ball_y + (ball_y - self.g_center_y)*t
            self.line_up_r_x = self.p_area_up_x + (ball_x - self.g_center_x)*t
            self.line_up_y = (self.g_up_y - ball_y)/(self.g_up_x - ball_x)*(self.p_area_up_x - ball_x) + ball_y
            self.line_up_x = self.p_area_up_x
            c_2 = self.line_up_y*(self.g_center_y - ball_y) + self.line_up_x*(self.g_center_x - ball_x)
            self.line_down_r_y = (b_2*c_5 - b_5*c_2)/(a_2*b_5 - a_5*b_2) + (ball_y - self.g_center_y)*t
            self.line_down_r_x = (a_2*c_5 - a_5*c_2)/(a_5*b_2 - a_2*b_5) + (ball_x - self.g_center_x)*t
            self.L_a = abs(a_2*ball_y + b_2*ball_x + c_2)/math.sqrt(a_2**2 + b_2**2)

        # # 最上部
        elif ball_x >= (self.g_up_x - self.p_area_up_x)/(self.g_up_y - self.p_area_up_x)*(ball_y - self.g_up_y) + self.g_up_x:
            self.line_up_r_y = (b_6*c_4 - b_4*c_6)/(a_6*b_4 - a_4*b_6) + (ball_y - self.g_center_y)*t
            self.line_up_r_x = (a_6*c_4 - a_4*c_6)/(a_4*b_6 - a_6*b_4) + (ball_x - self.g_center_x)*t
            self.line_down_r_y = (b_6*c_5 - b_5*c_6)/(a_6*b_5 - a_5*b_6) + (ball_y - self.g_center_y)*t
            self.line_down_r_x = (a_6*c_5 - a_5*c_6)/(a_5*b_6 - a_6*b_5) + (ball_x - self.g_center_x)*t
            self.L_a = abs(a_6*ball_y + b_6*ball_x + c_6)/math.sqrt(a_6**2 + b_6**2)

        # その他
        else:
            self.line_up_r_x = self.p_area_up_x + self.offset_r
            self.line_up_r_y = self.g_up_y/2
            self.line_down_r_x = self.p_area_down_x + self.offset_r
            self.line_down_r_y = self.g_down_y/2

        # ここまでが壁の基本位置計算
        # ここからがロボットの移動を考慮した位置補正と壁をニアorファーサイドに寄せる計算

        # ボールが壁に到達するまでに移動可能な距離の計算
        tmp = (self.L_a/self.ball_MAX_SPEED - self.delay_time_ms/1000.0)
        if tmp > 0:
            self.L_G = self.robot_a*(tmp**2)/2.0
        else:
            self.L_G = 0

        # ボールがハーフラインよりも敵陣側（壁が一台）かつ1台で守れる範囲：パターン1
        if (ball_x > 0.5) and (((self.line_up_r_y - self.line_down_r_y)**2 + (self.line_up_r_x - self.line_down_r_x)**2) <= 4.0*((self.L_G + self.robot_r)**2)):
            def1_pos_y = (self.line_up_r_y + self.line_down_r_y)/2.0
            def1_pos_x = (self.line_up_r_x + self.line_down_r_x)/2.0
            def2_pos_y = functions.calculate_internal_dividing_point_vector_args(self.ball_params.get_current_position(), config.GOAL_CENTER, 1, 1)[1]
            def2_pos_x = functions.calculate_internal_dividing_point_vector_args(self.ball_params.get_current_position(), config.GOAL_CENTER, 1, 1)[0]

        # ボールがハーフラインよりも味方陣側（壁が二台）かつ2台で守れる範囲：パターン2-1,2
        elif (ball_x <= 0) and (((self.line_up_r_y - self.line_down_r_y)**2 + (self.line_up_r_x - self.line_down_r_x)**2) <= 16.0*((self.L_G + self.robot_r)**2)):
            y_R = (3.0*self.line_down_r_y + self.line_up_r_y)/4
            x_R = (3.0*self.line_down_r_x + self.line_up_r_x)/4
            y_L = (self.line_down_r_y + 3.0*self.line_up_r_y)/4
            x_L = (self.line_down_r_x + 3.0*self.line_up_r_x)/4
            # 2台がぶつからない場合
            if (y_R - y_L)**2 + (x_R - x_L)**2 >= 4.0*(self.robot_r**2):
                def1_pos_y = y_L
                def1_pos_x = x_L
                def2_pos_y = y_R
                def2_pos_x = x_R
            # 2台がぶつかるのでずらす
            else:
                t_1 = self.robot_r/math.sqrt((self.line_down_r_y - self.line_up_r_y)**2 + (self.line_down_r_x - self.line_up_r_x)**2)
                def1_pos_y = (self.line_up_r_y + self.line_down_r_y)/2.0 + (self.line_up_r_y - self.line_down_r_y)*t_1
                def1_pos_x = (self.line_up_r_x + self.line_down_r_x)/2.0 + (self.line_up_r_x - self.line_down_r_x)*t_1
                def2_pos_y = (self.line_up_r_y + self.line_down_r_y)/2.0 - (self.line_up_r_y - self.line_down_r_y)*t_1
                def2_pos_x = (self.line_up_r_x + self.line_down_r_x)/2.0 - (self.line_up_r_x - self.line_down_r_x)*t_1

        # 壁が1台かつ1台で守れない or 壁が2台かつ2台で守れない：パターン3-1,2,3,4
        else:
            t_2 = (self.L_G + self.robot_r)/math.sqrt((self.line_up_r_y - self.line_down_r_y)**2 + (self.line_up_r_x - self.line_down_r_x)**2)
            # 1台の時
            if ball_x > 0.5:
                # 右サイドにボールがある
                if ball_y > 0:
                    def1_pos_y = self.line_down_r_y + (self.line_up_r_y - self.line_down_r_y)*t_2
                    def1_pos_x = self.line_up_r_x + (self.line_down_r_x - self.line_up_r_x)*t_2
                    def2_pos_y = functions.calculate_internal_dividing_point_vector_args(self.ball_params.get_current_position(), config.GOAL_CENTER, 1, 1)[1]
                    def2_pos_x = functions.calculate_internal_dividing_point_vector_args(self.ball_params.get_current_position(), config.GOAL_CENTER, 1, 1)[0]
                # 左サイドにボールがある
                else:
                    def1_pos_y = self.line_up_r_y + (self.line_down_r_y - self.line_up_r_y)*t_2
                    def1_pos_x = self.line_down_r_x + (self.line_up_r_x - self.line_down_r_x)*t_2
                    def2_pos_y = functions.calculate_internal_dividing_point_vector_args(self.ball_params.get_current_position(), config.GOAL_CENTER, 1, 1)[1]
                    def2_pos_x = functions.calculate_internal_dividing_point_vector_args(self.ball_params.get_current_position(), config.GOAL_CENTER, 1, 1)[0]
            # 2台の時
            else:
                # 右サイドにボールがある
                if ball_y > 0:
                    def2_pos_y = self.line_down_r_y + (self.line_up_r_y - self.line_down_r_y)*t_2
                    def2_pos_x = self.line_down_r_x + (self.line_up_r_x - self.line_down_r_x)*t_2
                    def1_pos_y = self.line_down_r_y + 3.0*(self.line_up_r_y - self.line_down_r_y)*t_2
                    def1_pos_x = self.line_down_r_x + 3.0*(self.line_up_r_x - self.line_down_r_x)*t_2
                # 左サイドにボールがある
                else:
                    def1_pos_y = self.line_up_r_y + (self.line_down_r_y - self.line_up_r_y)*t_2
                    def1_pos_x = self.line_up_r_x + (self.line_down_r_x - self.line_up_r_x)*t_2
                    def2_pos_y = self.line_up_r_y + 3.0*(self.line_down_r_y - self.line_up_r_y)*t_2
                    def2_pos_x = self.line_up_r_x + 3.0*(self.line_down_r_x - self.line_up_r_x)*t_2

        # 念の為クリップ
        def1_pos_x = np.clip(def1_pos_x, -6.0, 6.0)
        def1_pos_y = np.clip(def1_pos_y, -4.5, 4.5)
        def2_pos_x = np.clip(def2_pos_x, -6.0, 6.0)
        def2_pos_y = np.clip(def2_pos_y, -4.5, 4.5)
        # パブリッシュ用の変数に代入
        self.def_pos.def1_pos_x = def1_pos_x
        self.def_pos.def1_pos_y = def1_pos_y
        self.def_pos.def2_pos_x = def2_pos_x
        self.def_pos.def2_pos_y = def2_pos_y

def run_calculation():
    calcuration = Calculation()
    pub = Publisher()

    loop_rate = rospy.Rate(WORLD_LOOP_RATE)

    rospy.loginfo("start calculation node")

    while not rospy.is_shutdown():
        calcuration.calc_ball_line()
        calcuration.calc_def_pos()
        pub.ball_params_publisher(calcuration.ball_sub_params)
        pub.def_pos_publisher(calcuration.def_pos)
        loop_rate.sleep()

if __name__ == "__main__":
    while True and not rospy.is_shutdown():
        try:
            run_calculation()
        except:
            import traceback
            traceback.print_exc()
            if rospy.get_param("is_test", False):
                break
