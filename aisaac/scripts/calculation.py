#!/usr/bin/env  python
# coding:utf-8
import math
import rospy
import numpy as np
import sys
import time
import robot
from nav_msgs.msg import Odometry
from aisaac.msg import Ball_sub_params
import tf
import time
from statistics import mean, median,variance,stdev

WORLD_LOOP_RATE = 60.

"""
主に共通した計算処理などを担当する
"""
class Ball():
    def __init__(self):
        self.ball_pos_x = 0
        self.ball_pos_y = 0
        self.ball_pos_theta = 0
        self.ball_vel_x = 0
        self.ball_vel_y = 0
        
        self.team_color = str(rospy.get_param('/blue/team_color'))
        
        self.ball_sub_params = Ball_sub_params()
        
        self.time1 = 0
        self.time2 = 0
        self.time_duration = 0
        self.hz1 = 0

        self.ball_sub_params_pub = rospy.Publisher("/" + self.team_color + "/ball_sub_params", Ball_sub_params, queue_size=10)

    def odom_callback(self, msg):
        #self.time2 = self.time1
        #self.time1 = msg.header.stamp
        #self.time_duration = rospy.Duration()
        #self.time_duration = self.time1 - self.time2
        #self.hz1 = 1. / (self.time_duration.nsecs/100000000)
        #rospy.loginfo(self.time_duration.nsecs)

        self.time2 = self.time1
        self.time1 = time.time()
        self.time_duration = self.time1 - self.time2
        self.ball_pos_x = msg.pose.pose.position.x
        self.ball_pos_y = msg.pose.pose.position.y
        self.ball_pos_theta = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.ball_vel_x = msg.twist.twist.linear.x
        self.ball_vel_y = msg.twist.twist.linear.y

        #rospy.loginfo("%.4f\t%.4f\t%.4f\t%.4f\t",1/self.time_duration,self.time_duration,self.time1,self.time2)

    def ball_params_publisher(self):
        self.ball_sub_params_pub.publish(self.ball_sub_params)

class Calculation():
    def __init__(self):
        rospy.init_node("Calculation_node")
        self.robot_color = str(rospy.get_param("~robot_color"))

        self.ball_params = Ball()

        self.ball_frame = 10 #ボールの軌道直線フィッティングと速度の計算フレーム
        self.same_pos_count = 0
        self.ball_pos_count = 0
        self.calc_flag = False
        self.ball_pos_x_array = np.array([0.0]*self.ball_frame)
        self.ball_pos_y_array = np.array([0.0]*self.ball_frame)
        self.ball_vel_x_array = np.array([0.0]*self.ball_frame)
        self.ball_vel_y_array = np.array([0.0]*self.ball_frame)
        self.ball_vel_time_array = np.array([0.0]*self.ball_frame)

        self.ball_vel_x_a = 0.
        self.ball_vel_x_b = 0.
        self.ball_vel_y_a = 0.
        self.ball_vel_y_b = 0.
        self.ball_stop_time_x = 0.
        self.ball_stop_time_y = 0.

    def odom_listener(self):
        rospy.Subscriber("/" + self.robot_color + "/ball_observer/estimation", Odometry, self.ball_params.odom_callback)

    def reg1dim(self, x, y, n):
        x = np.clip(x,-5,5)
        y = np.clip(y,-5,5)
        #np.dot(x, y)
        a = np.clip(((np.dot(x, y) - y.sum()*x.sum()/n) / ((x**2.).sum() - x.sum()**2./n)),-1.0e+3,1.0e+3)
        b = np.clip((y.sum() - a * x.sum())/n,-1.0e+3,1.0e+3)
        return a, b

    def calc_ball_line(self):
        #直近nフレームの座標を取得
        if self.ball_pos_count < self.ball_frame:
            self.ball_pos_x_array[self.ball_pos_count] = self.ball_params.ball_pos_x
            self.ball_pos_y_array[self.ball_pos_count] = self.ball_params.ball_pos_y
            self.ball_vel_x_array[self.ball_pos_count] = self.ball_params.ball_vel_x
            self.ball_vel_y_array[self.ball_pos_count] = self.ball_params.ball_vel_y
            self.ball_vel_time_array[self.ball_pos_count] = 1./WORLD_LOOP_RATE * self.ball_pos_count
            if self.ball_pos_count > 0:
                if abs(self.ball_pos_x_array[self.ball_pos_count-1]-self.ball_pos_x_array[self.ball_pos_count]) < 0.001 and abs(self.ball_pos_y_array[self.ball_pos_count-1]-self.ball_pos_y_array[self.ball_pos_count]) < 0.001:
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
            self.ball_vel_x_array = np.roll(self.ball_vel_x_array,-1)
            self.ball_vel_y_array = np.roll(self.ball_vel_y_array,-1)
            self.ball_pos_x_array[self.ball_pos_count-1] = self.ball_params.ball_pos_x
            self.ball_pos_y_array[self.ball_pos_count-1] = self.ball_params.ball_pos_y
            self.ball_vel_x_array[self.ball_pos_count-1] = self.ball_params.ball_vel_x
            self.ball_vel_y_array[self.ball_pos_count-1] = self.ball_params.ball_vel_y
            if abs(self.ball_pos_x_array[self.ball_pos_count-2]-self.ball_pos_x_array[self.ball_pos_count-1]) < 0.001 or abs(self.ball_pos_y_array[self.ball_pos_count-2]-self.ball_pos_y_array[self.ball_pos_count-1]) < 0.001:
                self.same_pos_count+=1
                if self.same_pos_count > self.ball_frame/2:
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
                for i in range(0,self.ball_frame):
                    self.ball_pos_x_array[i] = 0
                    self.ball_pos_y_array[i] = 0

        #print(self.ball_pos_x_array)
        #print(self.ball_pos_count)
        #print(self.same_pos_count)
        if self.calc_flag == True:
            self.ball_params.ball_sub_params.a, self.ball_params.ball_sub_params.b = self.reg1dim(self.ball_pos_x_array, self.ball_pos_y_array, self.ball_pos_count)
            self.ball_vel_x_a, self.ball_vel_x_b = self.reg1dim(self.ball_vel_x_array, self.ball_vel_time_array, self.ball_pos_count)
            self.ball_vel_y_a, self.ball_vel_y_b = self.reg1dim(self.ball_vel_y_array, self.ball_vel_time_array, self.ball_pos_count)
            #self.ball_params.ball_sub_params.a, self.ball_params.ball_sub_params.b = self.reg1dim(self.ball_vel_x_array, self.ball_vel_time_array, self.ball_pos_count)
            """ self.ball_params.ball_sub_params.future_x = 
            self.ball_params.ball_sub_params.future_y """
            #rospy.loginfo("vel_x_a:%f\tvel_x_b:%f",self.ball_vel_x_a, self.ball_vel_x_b)

            #ボールの予想停止位置を計算
            #x,y方向の現在の速度を最小二乗法で求めた直線から計算→式が違う、速度推定が必要
            #ball_fit_vel_x = self.ball_vel_x_a*self.ball_vel_time_array[self.ball_pos_count-1] + self.ball_vel_x_b
            #ball_fit_vel_y = self.ball_vel_y_a*self.ball_vel_time_array[self.ball_pos_count-1] + self.ball_vel_y_b
            #とりあえず現在速度を使う
            ball_fit_vel_x = self.ball_params.ball_vel_x
            ball_fit_vel_y = self.ball_params.ball_vel_y
            #停止するまでの時間を現在の速度と傾きから計算
            if self.ball_vel_x_a != 0 and self.ball_vel_y_a != 0:
                self.ball_stop_time_x = -(ball_fit_vel_x / self.ball_vel_x_a)
                self.ball_stop_time_y = -(ball_fit_vel_y / self.ball_vel_y_a)
                if self.ball_stop_time_x <= 0 or self.ball_stop_time_y <= 0:
                    """ self.ball_params.ball_sub_params.future_x = 0
                    self.ball_params.ball_sub_params.future_y = 0 """
                else:
                    self.ball_params.ball_sub_params.future_x = self.ball_params.ball_pos_x + ball_fit_vel_x*self.ball_stop_time_x + 1/2*self.ball_vel_x_a*self.ball_stop_time_x**2
                    self.ball_params.ball_sub_params.future_y = self.ball_params.ball_pos_y + ball_fit_vel_y*self.ball_stop_time_y + 1/2*self.ball_vel_y_a*self.ball_stop_time_y**2
                    self.ball_params.ball_sub_params.future_x = np.clip(self.ball_params.ball_sub_params.future_x,-5,5)
                    self.ball_params.ball_sub_params.future_y = np.clip(self.ball_params.ball_sub_params.future_y,-5,5)
                    #rospy.loginfo("t=(%.3f,%.3f)\t(f_x:n_x)=(%.3f:%.3f)\t(f_y:n_y)=(%.3f:%.3f)",self.ball_stop_time_x,self.ball_stop_time_y,self.ball_params.ball_sub_params.future_x, self.ball_params.ball_pos_x, self.ball_params.ball_sub_params.future_y, self.ball_params.ball_pos_y)
        """ else:
            for i in range(0,self.ball_frame):
                self.ball_pos_x_array[i] = 0
                self.ball_pos_y_array[i] = 0 
                self.ball_vel_x_array[i] = 0
                self.ball_vel_y_array[i] = 0 """
        
        #print(self.ball_stop_time_x,self.ball_stop_time_y)
        rospy.loginfo("f=%d\tt=(%.3f,%.3f)\t(f_x:n_x)=(%.3f:%.3f)\t(f_y:n_y)=(%.3f:%.3f)",self.calc_flag,self.ball_stop_time_x,self.ball_stop_time_y,self.ball_params.ball_sub_params.future_x, self.ball_params.ball_pos_x, self.ball_params.ball_sub_params.future_y, self.ball_params.ball_pos_y)

if __name__ == "__main__":
    a = Calculation()
    
    a.odom_listener()

    loop_rate = rospy.Rate(WORLD_LOOP_RATE)

    #time3 = 0
    #time4 = 0

    print("start calculation node")

    try:
        while not rospy.is_shutdown():
            #print(a.ball_params.ball_pos_x,a.ball_params.ball_pos_y)

            #time4 = time3
            #time3 = time.time()
            #hz2 = 1 / (time3 - time4)
            #print(hz2:hz2)

            #rospy.loginfo("hz1:%f\thz2:%f", a.ball_params.hz1, hz2)
            #rospy.loginfo("%s",a.ball_params.header_time)

            #rospy.spinOnce()

            a.calc_ball_line()
            a.ball_params.ball_params_publisher()
            loop_rate.sleep()

    except:
        print("error")
