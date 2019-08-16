#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from aisaac.msg import Status
from aisaac.srv import pid
import time
import pandas
import math
import numpy as np
import tf
from nav_msgs.msg import Odometry
import functions
import config
from bayes_opt import BayesianOptimization
import ipdb

global mode

class Main(object):
    def main(self, Kpv=None, Kpr=None, Kdv=None, Kdr=None):

        robot_id = str(0)
        robot_color = 'blue'

        status_topic_name = '/'+robot_color+'/robot_'+robot_id+'/status'
        self.status_topic_publisher = rospy.Publisher(
            status_topic_name, Status, queue_size=1)

        pid_service_name = '/'+robot_color+'/robot_'+robot_id+'/set_pid'
        self.pid_service_proxy = rospy.ServiceProxy(pid_service_name, pid)
        rospy.wait_for_service(pid_service_name)

        self.robot = {}

        rospy.wait_for_message(
            '/'+robot_color+'/robot_'+robot_id+'/odom', Odometry)
        self.odom_listener = rospy.Subscriber(
            '/'+robot_color+'/robot_'+robot_id+'/odom', Odometry, self.odom_listener_callback)
        rospy.wait_for_message(
            '/'+robot_color+'/robot_'+robot_id+'/odom', Odometry)

        # {'Kdv': 5.436618301696918, 'Kpv': 3.111774552866889}
        if Kpv is None:
            Kpv = 3.111774552866889
        if Kpr is None:
            Kpr = 2.0
        if Kdv is None:
            Kdv = 5.436618301696918
        if Kdr is None:
            Kdr = 2.0
        # kdr 4.78     |  5.037    

        next_param = {
            'Kpv': Kpv,
            'Kpr': Kpr,
            'Kdv': Kdv,
            'Kdr': Kdr,
        }

        status = Status()
        status.status = "move_linear"
        status.pid_goal_pos_x = 0.0
        status.pid_goal_pos_y = 1.0
        status.pid_goal_theta = 0.0

        self.pid_service_proxy(2.0,
                               2.0,
                               2.0,
                               2.0)
        self.publish(status)

        rospy.sleep(3.0)

        self.pid_service_proxy(next_param['Kpv'],
                               next_param['Kpr'],
                               next_param['Kdv'],
                               next_param['Kdr'])


        loop_rate = rospy.Rate(config.ROBOT_LOOP_RATE)

        total_elapsed_time = 0.0

        # x, y, theta
        global mode
        if mode == "position":
            positions = [
                [0.0, 0.0, 0.3 * np.pi],
                [0.0, 1.0, 0.9 * np.pi],
                [0.0, -1.0, 1.8 * np.pi],
                [0.0, 2.0, 0.6 * np.pi],
            ]
        else:
            positions = [
                [0.0, 1.0, 0.3 * np.pi],
                [0.0, 1.0, 0.9 * np.pi],
                [0.0, 1.0, 1.8 * np.pi],
                [0.0, 1.0, 0.6 * np.pi],
            ]

        total_average_pos = 0.0
        total_average_rot = 0.0
        for pos in positions:
            status.pid_goal_pos_x = pos[0]
            status.pid_goal_pos_y = pos[1]
            status.pid_goal_theta = pos[2]

            # print(str(status))

            list_length = 60
            hantei_list_pos = [10.0 for _ in range(list_length)]
            hantei_list_rot = [1.0 * np.pi for _ in range(list_length)]

            area_pos = 0.02
            area_rot = np.pi * 3.0 / 180.0

            start_time = rospy.Time.now()
            # Start measuring

            time_limit = 4.0
            additional_cost = 0.0

            self.publish(status)
            while not rospy.is_shutdown():
                distance_to_target_pos = \
                    functions.distance_btw_two_points((status.pid_goal_pos_x, status.pid_goal_pos_y),
                                                      (self.robot['x'], self.robot['y']))

                distance_to_target_rot = self.diff_theta(status.pid_goal_theta, self.robot['theta'])

                hantei_list_pos.append(distance_to_target_pos)
                hantei_list_pos.pop(0)

                hantei_list_rot.append(distance_to_target_rot)
                hantei_list_rot.pop(0)

                average_pos = np.average(hantei_list_pos)
                average_rot = np.average(hantei_list_rot)
                # rospy.loginfo_throttle(1, "pos average: "+str(average_pos)+" rot average: "+str(average_rot))

                current_elapsed_time = rospy.Time.now() - start_time


                time_over = abs(current_elapsed_time.to_sec()) > time_limit

                if mode == "position":
                    condition = (average_pos < area_pos and average_rot < area_rot) or time_over
                else:
                    condition = average_rot < area_rot or time_over

                if condition:
                    # if current_elapsed_time.to_sec() > 3.0:
                    # total_average_pos = total_average_pos + average_pos
                    # total_average_rot = total_average_rot + average_rot
                    if time_over:
                        # 評価関数が非線形だろうが不連続だろうがそんなものは気にしない
                        additional_cost = 10 * (average_pos + average_rot)

                    break

                loop_rate.sleep()

            end_time = rospy.Time.now()
            elapsed_time = (end_time - start_time).to_sec() + additional_cost
            total_elapsed_time = total_elapsed_time + elapsed_time

            # print("time: " + str(elapsed_time) + " sec")
            # print("")

            if rospy.is_shutdown():
                break

        # print("total_elapsed_time:" + str(total_elapsed_time))
        return 10.0 / total_elapsed_time
        # return 1.0 / (total_average_pos + total_average_rot)

    def diff_theta(self, goal_pos_theta, current_theta):

        d_theta = goal_pos_theta - current_theta
        if d_theta < 0 and abs(d_theta) > np.pi:
            d_theta = goal_pos_theta - (current_theta - 2 * np.pi)
        if d_theta > 0 and abs(d_theta) > np.pi:
            d_theta = (goal_pos_theta - 2 * np.pi) - current_theta

        return abs(d_theta)


    def publish(self, msg):
        for _ in range(100):
            self.status_topic_publisher.publish(msg)

    def odom_listener_callback(self, msg):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x,
                                                            msg.pose.pose.orientation.y,
                                                            msg.pose.pose.orientation.z,
                                                            msg.pose.pose.orientation.w))
        self.robot['x'] = robot_x
        self.robot['y'] = robot_y
        self.robot['theta'] = robot_t[2]


if __name__ == '__main__':
    mode = "position"
    # mode = "rotation"

    rospy.init_node("autotuner")
    program_start_time = rospy.Time.now()

    print("AutotuneMode: " + mode)

    main = Main()

    if mode == "position":
        pbounds = {
                   'Kpv': (1, 10),
                   'Kdv': (1, 10),
                   }
    else:
        pbounds = {
                   'Kpr': (1, 10),
                   'Kdr': (1, 10),
                   }
    optimizer = BayesianOptimization(
            f=main.main,
            pbounds=pbounds,
            random_state=1
    )
    optimizer.maximize(init_points=4, n_iter=20)
    print("max"+str(optimizer.max))
    print("res"+str(optimizer.res))

    program_end_time = rospy.Time.now()
    print("Elapsed time: " + str((program_end_time - program_end_time).to_sec()) + " sec.")
