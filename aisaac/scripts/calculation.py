#!/usr/bin/env  python
# coding:utf-8
import math
import rospy
import numpy as np
import sys
import time
import robot
from nav_msgs.msg import Odometry
import tf
import time
from statistics import mean, median,variance,stdev
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout

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
        
        self.ball_sub_params = Int32MultiArray()
        
        self.time1 = 0
        self.time2 = 0
        self.hz1 = 0

        self.ball_sub_params_pub = rospy.Publisher("/" + self.team_color + "/ball_sub_params", Int32MultiArray, queue_size=10)

    def odom_callback(self, msg):
        #self.time2 = self.time1
        #self.time1 = msg.header.stamp
        #self.time_duration = rospy.Duration()
        #elf.time_duration = self.time1 - self.time2
        #self.hz1 = 1. / (self.time_duration.nsecs/100000000)
        #rospy.loginfo(self.time_duration.nsecs)
        self.ball_pos_x = msg.pose.pose.position.x
        self.ball_pos_y = msg.pose.pose.position.y
        self.ball_pos_theta = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.ball_vel_x = msg.twist.twist.linear.x
        self.ball_vel_y = msg.twist.twist.linear.y

    def ball_params_publisher(self):
        self.ball_sub_params_pub.publish(self.ball_sub_params)

class Calculation():
    def __init__(self):
        rospy.init_node("Calculation_node")
        self.robot_color = str(rospy.get_param("~robot_color"))

        self.ball_params = Ball()

        self.ball_frame = 60 #ボールの軌道直線フィッティングと速度の計算フレーム
        self.ball_pos_count = 0
        self.ball_pos_x_array = np.array([0.0]*self.ball_frame)
        self.ball_pos_y_array = np.array([0.0]*self.ball_frame)

    def odom_listener(self):
        rospy.Subscriber("/" + self.robot_color + "/ball_observer/estimation", Odometry, self.ball_params.odom_callback)

    def reg1dim(self, x, y, n):
        x = np.clip(x,-5,5)
        y = np.clip(y,-5,5)
        a = ((np.dot(x, y)- y.sum() * x.sum()/n) / ((x ** 2).sum() - x.sum()**2 / n))
        b = (y.sum() - a * x.sum())/n
        a = np.clip(a,-1.0e+308,1.0e+308)
        b = np.clip(b,-1.0e+308,1.0e+308)
        return a, b

    def calc_ball_line(self):
        #直近60フレームの座標を取得
        if self.ball_pos_count < self.ball_frame:
            self.ball_pos_x_array[self.ball_pos_count] = self.ball_params.ball_pos_x
            self.ball_pos_y_array[self.ball_pos_count] = self.ball_params.ball_pos_y
            self.ball_pos_count+=1
        else:
            self.ball_pos_x_array = np.roll(self.ball_pos_x_array,-1)
            self.ball_pos_y_array = np.roll(self.ball_pos_y_array,-1)
            self.ball_pos_x_array[self.ball_pos_count-1] = self.ball_params.ball_pos_x
            self.ball_pos_y_array[self.ball_pos_count-1] = self.ball_params.ball_pos_y
            #x,y座標の分散を計算
            x_variance = variance(self.ball_pos_x_array)
            y_variance = variance(self.ball_pos_y_array)
            #print(x_variance,y_variance)
            #分散が1より大きかったらカウントリセット
            if x_variance > 1 or y_variance > 1:
                self.ball_pos_count = 0
                for i in range(0,self.ball_frame):
                    self.ball_pos_x_array[i] = 0
                    self.ball_pos_y_array[i] = 0

        if self.ball_pos_count > 0:
            a, b = self.reg1dim(self.ball_pos_x_array, self.ball_pos_y_array, self.ball_pos_count)
            rospy.loginfo("%f\t%f",a,b)


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
