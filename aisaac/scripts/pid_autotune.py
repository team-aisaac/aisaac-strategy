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

class Main(object):
    def main(self, Kpv, Kdv):
        rospy.init_node("autotuner")

        status_topic_name = '/blue/robot_1/status'
        self.status_topic_publisher = rospy.Publisher(
            status_topic_name, Status, queue_size=1)

        pid_service_name = '/blue/robot_1/set_pid'
        self.pid_service_proxy = rospy.ServiceProxy(pid_service_name, pid)
        rospy.wait_for_service(pid_service_name)

        self.robot = {}
        rospy.wait_for_message("/blue/robot_1/odom", Odometry)
        self.odom_listener = rospy.Subscriber(
            "/blue/robot_1/odom", Odometry, self.odom_listener_callback)
        rospy.wait_for_message("/blue/robot_1/odom", Odometry)

        next_param = {
            'Kpv': Kpv,
            'Kpr': 3.0,
            'Kdv': Kdv,
            'Kdr': 3.0,
        }

        self.pid_service_proxy(next_param['Kpv'],
                               next_param['Kpr'],
                               next_param['Kdv'],
                               next_param['Kdr'])

        status = Status()
        status.status = "move_linear"
        status.pid_goal_pos_x = 0.0
        status.pid_goal_pos_y = 2.0
        status.pid_goal_theta = 0.0

        loop_rate = rospy.Rate(config.ROBOT_LOOP_RATE)

        total_elapsed_time = 0.0

        for _ in range(4):
            status.pid_goal_pos_y = status.pid_goal_pos_y * -1

            if status.pid_goal_pos_y > 0.0:
                status.pid_goal_theta = 0.5 * np.pi
            else:
                status.pid_goal_theta = 1.5 * np.pi

            print(str(status))


            distance_to_target_pos = functions.distance_btw_two_points((status.pid_goal_pos_x, status.pid_goal_pos_y),
                                                                       (self.robot['x'], self.robot['y']))
            hantei_list = [distance_to_target_pos for _ in range(60)]
            area = 0.02

            start_time = rospy.Time.now()
            # Start measuring
            self.publish(status)
            while not rospy.is_shutdown():
                distance_to_target_pos = functions.distance_btw_two_points((status.pid_goal_pos_x, status.pid_goal_pos_y),
                                                                           (self.robot['x'], self.robot['y']))
                # distance_to_target_rot = status.pid_goal_theta - self.robot['theta']
                distance_to_target_rot = 0.0

                hantei_list.append(distance_to_target_pos+distance_to_target_rot)
                hantei_list.pop(0)

                average = np.average(hantei_list)
                rospy.loginfo_throttle(1, average)

                current_elapsed_time = rospy.Time.now() - start_time
                if average < area or current_elapsed_time.to_sec() > 20.0:
                    break

                loop_rate.sleep()

            end_time = rospy.Time.now()
            elapsed_time = (end_time - start_time).to_sec()
            total_elapsed_time = total_elapsed_time + elapsed_time

            print("time: " + str(elapsed_time) + " sec")
            print("")

            if rospy.is_shutdown():
                break

        print("total_elapsed_time:" + str(total_elapsed_time))
        return 1.0 / total_elapsed_time

    def publish(self, msg):
        for _ in range(100):
            self.status_topic_publisher.publish(msg)

    def odom_listener_callback(self, msg):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_t = tf.transformations.euler_from_quaternion(
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        # robot_v_x = msg.twist.twist.linear.x
        # robot_v_y = msg.twist.twist.linear.y
        # robot_v_t = msg.twist.twist.linear.z

        self.robot['x'] = robot_x
        self.robot['y'] = robot_y
        self.robot['theta'] = robot_t[2]


if __name__ == '__main__':
    main = Main()
    pbounds = {'Kpv': (1, 10), 'Kdv': (1,10)}
    optimizer = BayesianOptimization(
            f=main.main,
            pbounds=pbounds,
            random_state=1
    )
    optimizer.maximize(init_points=2, n_iter=20)
    ipdb.set_trace()
    # Best
    # {'params': {'Kdv': 1.9759837181620452, 'Kpv': 3.615645812128088}, 'target': 0.06376533043101164}

    # Result
    # [{'params': {'Kdv': 4.753198042323167, 'Kpv': 7.482920440979423}, 'target': 0.04655285293096144}, {'params': {'Kdv': 1.001029373356104, 'Kpv': 3.7209931536865577}, 'target': 0.06224471443775683}, {'params': {'Kdv': 10.0, 'Kpv': 1.0}, 'target': 0.029674383820484745}, {'params': {'Kdv': 1.0, 'Kpv': 1.0}, 'target': 0.03434578555176569}, {'params': {'Kdv': 1.0, 'Kpv': 10.0}, 'target': 0.03243422521722526}, {'params': {'Kdv': 10.0, 'Kpv': 10.0}, 'target': 0.03958088597475348}, {'params': {'Kdv': 10.0, 'Kpv': 5.679552045353724}, 'target': 0.05597603566544504}, {'params': {'Kdv': 1.0, 'Kpv': 5.890091558994576}, 'target': 0.045979381876197226}, {'params': {'Kdv': 5.426682312212606, 'Kpv': 2.547491243612181}, 'target': 0.0585988878589267}, {'params': {'Kdv': 7.415316393319363, 'Kpv': 4.574300835655964}, 'target': 0.0573657447553172}, {'params': {'Kdv': 3.2492939682073922, 'Kpv': 3.8140512829793227}, 'target': 0.0631601386171141}, {'params': {'Kdv': 6.294295083526826, 'Kpv': 9.996673457994227}, 'target': 0.04106946858325733}, {'params': {'Kdv': 2.463673977213546, 'Kpv': 3.0447862602037707}, 'target': 0.06323699532087611}, {'params': {'Kdv': 4.727356247883792, 'Kpv': 3.897669188718873}, 'target': 0.06168303477057576}, {'params': {'Kdv': 2.2758822487103063, 'Kpv': 3.6429485159888575}, 'target': 0.062377155540970695}, {'params': {'Kdv': 3.5180743269966985, 'Kpv': 3.110095733363163}, 'target': 0.059764688396751975}, {'params': {'Kdv': 9.997826292565573, 'Kpv': 3.892316161869426}, 'target': 0.058599613226648764}, {'params': {'Kdv': 1.9759837181620452, 'Kpv': 3.615645812128088}, 'target': 0.06376533043101164}, {'params': {'Kdv': 1.8417047306916379, 'Kpv': 3.5016872356259126}, 'target': 0.061417078057509754}, {'params': {'Kdv': 2.6561728012242414, 'Kpv': 3.9213410107425295}, 'target': 0.059553632015326034}, {'params': {'Kdv': 1.0128747435673153, 'Kpv': 3.2957223288489983}, 'target': 0.06237396546179895}, {'params': {'Kdv': 5.5670228992286965, 'Kpv': 3.768682415995859}, 'target': 0.059645860376855896}]
