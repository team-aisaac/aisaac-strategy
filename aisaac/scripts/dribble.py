#!/usr/bin/env  python
# coding:utf-8
import itertools
import math
import rospy
import numpy as np
from consai_msgs.msg import Pose
from consai_msgs.msg import robot_commands
#from aisaac.srv import Kick
from aisaac.msg import Status
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import entity
import matplotlib.pyplot as plt
from matplotlib import animation
import functions
from robot_functions import RobotPid
ROBOT_LOOP_RATE = 60.
DT =  1. / ROBOT_LOOP_RATE
COEFFICIENT = 1.
MAX_SPEED =  3.7


class Dribble(RobotPid):
    def __init__(self, robot_params, ball_params, cmd, command_pub):
        super(Dribble, self).__init__(robot_params, ball_params, cmd, command_pub)
        #self.max_dribble_time = 1.5

    def calculate_vector(self, vel_x_before, vel_y_before, vel_x_after, vel_y_after):
        acceleration_x = (vel_x_after - vel_x_before) / DT  #ロボットの加速度x = ホールにかかる加速度と見ている
        acceleration_y = (vel_y_after - vel_y_before) / DT　#ロボットの加速度y = ホールにかかる加速度と見ている
        # 遠心力の計算
        centrifugalforce_x, centrifugalforce_y = functions.calculate_centrifugalforce(vel_x_before, vel_y_before, vel_x_after, vel_y_after)
        # 遠心力方向の加速度と進行方向の加速度の合成ベクトル(ボールが受ける加速度)の逆ベクトル
        composite_vector_x = -1 * acceleration_x * COEFFICIENT + centrifugalforce_x
        composite_vector_y = -1 * acceleration_y * COEFFICIENT + centrifugalforce_y
        # 目標角度とボールの加速度の逆ベクトル
        tan_acceleration = np.arctan2(acceleration_x, acceleration_y)
        # tan_
        if np.arctan2(composite_vector_x, composite_vector_y) > np.arctan2(acceleration_x, acceleration_y):

        vel_x_after / MAX_SPEED * composite_vector_x +

    def path_plan(self, goal_pos_x, goal_pos_y):
        self.recursion_count += 1
        collision = self.collision_Detection(goal_pos_x, goal_pos_y)
        if collision[0] and self.recursion_count < self.recursion_max:
            goal_pos_x, goal_pos_y = self.get_sub_goal(collision[1], collision[2], collision[3], collision[4], collision[5])
            self.path_plan(goal_pos_x, goal_pos_y)
        return goal_pos_x, goal_pos_y

    def pid_linear(self, goal_pos_x, goal_pos_y, goal_pos_theta):
        self.Kpv = 2.2
        self.Kpr = 4.
        self.Kdv = 5.
        self.Kdr = 3.

        if self.goal_pos_init_flag == True:
            self.recursion_count = 0
            self.next_pos_x, self.next_pos_y = self.pass_plan(goal_pos_x, goal_pos_y)

        d_x = self.next_pos_x - self.robot_params.current_x
        d_y = self.next_pos_y - self.robot_params.current_y
        d_theta = goal_pos_theta - self.robot_params.current_theta
        if self.goal_pos_init_flag:
            self.goal_pos_init_flag = False
            self.pid_d_x = 0.
            self.pid_d_y = 0.
            self.pid_d_theta = 0.
            self.pid_p_prev_x = d_x
            self.pid_p_prev_y = d_y
            self.pid_p_prev_theta = d_theta
        else:
            self.pid_d_x = d_x - self.pid_p_prev_x
            self.pid_d_y = d_y - self.pid_p_prev_y
            self.pid_d_theta = d_theta - self.pid_p_prev_theta

            self.pid_p_prev_x = d_x
            self.pid_p_prev_y = d_y
            self.pid_p_prev_theta = d_theta

        self.pid_p_x = d_x
        self.pid_p_y = d_y
        self.pid_p_theta = d_theta

        Vx = self.Kpv * self.pid_p_x + self.Kdv * self.pid_d_x / (1./ROBOT_LOOP_RATE)
        Vy = self.Kpv * self.pid_p_y + self.Kdv * self.pid_d_y / (1./ROBOT_LOOP_RATE)
        Vr = self.Kpr * self.pid_p_theta + self.Kdr * self.pid_d_theta / (1./ROBOT_LOOP_RATE)

        self.cmd.vel_surge = Vx*math.cos(self.robot_params.current_theta)+Vy*math.sin(self.robot_params.current_theta)
        self.cmd.vel_sway = -Vx*math.sin(self.robot_params.current_theta)+Vy*math.cos(self.robot_params.current_theta)
        self.cmd.omega = Vr
        self.command_pub.publish(self.cmd)

    def replan_timerCallback(self, event):
        self.goal_pos_init_flag = True
