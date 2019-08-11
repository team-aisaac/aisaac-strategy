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
    def main(self, Kpv=None, Kpr=None, Kdv=None, Kdr=None):
        rospy.init_node("autotuner")

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

        rospy.sleep(4.0)

        self.pid_service_proxy(next_param['Kpv'],
                               next_param['Kpr'],
                               next_param['Kdv'],
                               next_param['Kdr'])


        loop_rate = rospy.Rate(config.ROBOT_LOOP_RATE)

        total_elapsed_time = 0.0

        # x, y, theta
        # positions = [
        #     [0.0, 0.0, 0.3 * np.pi],
        #     [0.0, 2.0, 0.9 * np.pi],
        #     [0.0, -2.0, 1.8 * np.pi],
        #     [0.0, 4.0, 0.6 * np.pi],
        # ]
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

            distance_to_target_pos = \
                functions.distance_btw_two_points((status.pid_goal_pos_x, status.pid_goal_pos_y),
                                                  (self.robot['x'], self.robot['y']))
            distance_to_target_rot = self.diff_theta(status.pid_goal_pos_x, self.robot['theta'])

            list_length = 60
            hantei_list_pos = [distance_to_target_pos for _ in range(list_length)]
            hantei_list_rot = [distance_to_target_rot for _ in range(list_length)]
            area_pos = 0.02
            area_rot = np.pi * 3.0 / 180.0

            start_time = rospy.Time.now()
            # Start measuring

            time_limit = 5.0
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


                time_over = current_elapsed_time.to_sec() > time_limit
                # if (average_pos < area_pos and average_rot < area_rot) or time_over:
                if average_rot < area_rot or time_over:
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
    main = Main()
    pbounds = {
               # 'Kpv': (1, 10),
               'Kpr': (1, 10),
               # 'Kdv': (1, 10),
               'Kdr': (1, 10),
               }
    optimizer = BayesianOptimization(
            f=main.main,
            pbounds=pbounds,
            random_state=1
    )
    optimizer.maximize(init_points=4, n_iter=30)
    print("max"+str(optimizer.max))
    print("res"+str(optimizer.res))
    ipdb.set_trace()
    # Best
    # {'params': {'Kdv': 1.9759837181620452, 'Kpv': 3.615645812128088}, 'target': 0.06376533043101164}

    # Result
    # [{'params': {'Kdv': 4.753198042323167, 'Kpv': 7.482920440979423}, 'target': 0.04655285293096144}, {'params': {'Kdv': 1.001029373356104, 'Kpv': 3.7209931536865577}, 'target': 0.06224471443775683}, {'params': {'Kdv': 10.0, 'Kpv': 1.0}, 'target': 0.029674383820484745}, {'params': {'Kdv': 1.0, 'Kpv': 1.0}, 'target': 0.03434578555176569}, {'params': {'Kdv': 1.0, 'Kpv': 10.0}, 'target': 0.03243422521722526}, {'params': {'Kdv': 10.0, 'Kpv': 10.0}, 'target': 0.03958088597475348}, {'params': {'Kdv': 10.0, 'Kpv': 5.679552045353724}, 'target': 0.05597603566544504}, {'params': {'Kdv': 1.0, 'Kpv': 5.890091558994576}, 'target': 0.045979381876197226}, {'params': {'Kdv': 5.426682312212606, 'Kpv': 2.547491243612181}, 'target': 0.0585988878589267}, {'params': {'Kdv': 7.415316393319363, 'Kpv': 4.574300835655964}, 'target': 0.0573657447553172}, {'params': {'Kdv': 3.2492939682073922, 'Kpv': 3.8140512829793227}, 'target': 0.0631601386171141}, {'params': {'Kdv': 6.294295083526826, 'Kpv': 9.996673457994227}, 'target': 0.04106946858325733}, {'params': {'Kdv': 2.463673977213546, 'Kpv': 3.0447862602037707}, 'target': 0.06323699532087611}, {'params': {'Kdv': 4.727356247883792, 'Kpv': 3.897669188718873}, 'target': 0.06168303477057576}, {'params': {'Kdv': 2.2758822487103063, 'Kpv': 3.6429485159888575}, 'target': 0.062377155540970695}, {'params': {'Kdv': 3.5180743269966985, 'Kpv': 3.110095733363163}, 'target': 0.059764688396751975}, {'params': {'Kdv': 9.997826292565573, 'Kpv': 3.892316161869426}, 'target': 0.058599613226648764}, {'params': {'Kdv': 1.9759837181620452, 'Kpv': 3.615645812128088}, 'target': 0.06376533043101164}, {'params': {'Kdv': 1.8417047306916379, 'Kpv': 3.5016872356259126}, 'target': 0.061417078057509754}, {'params': {'Kdv': 2.6561728012242414, 'Kpv': 3.9213410107425295}, 'target': 0.059553632015326034}, {'params': {'Kdv': 1.0128747435673153, 'Kpv': 3.2957223288489983}, 'target': 0.06237396546179895}, {'params': {'Kdv': 5.5670228992286965, 'Kpv': 3.768682415995859}, 'target': 0.059645860376855896}]

    # best 4 parmas
    # {'params': {'Kdv': 9.527431239335174, 'Kpv': 1.8655161890744334, 'Kpr': 7.464509017573529, 'Kdr': 3.8946476422128846}, 'target': 0.4064310598967727}

    # result 4 params
    # [{'params': {'Kdv': 14.686165375401004, 'Kpv': 6.744318880004956, 'Kpr': 1.0021731215295528, 'Kdr': 8.923418089348907}, 'target': 0.23068321187450705}, {'params': {'Kdv': 2.7544333006071584, 'Kpv': 7.5656538138179075, 'Kpr': 4.538944016175748, 'Kdr': 3.7883619255251477}, 'target': 0.24346507333521702}, {'params': {'Kdv': 1.256283994487446, 'Kpv': 18.45328380311187, 'Kpr': 19.10345758051524, 'Kdr': 19.828334684111102}, 'target': 0.10142488552820839}, {'params': {'Kdv': 20.0, 'Kpv': 1.0, 'Kpr': 20.0, 'Kdr': 1.0}, 'target': 0.047949668374179216}, {'params': {'Kdv': 1.0, 'Kpv': 1.0, 'Kpr': 1.0, 'Kdr': 20.0}, 'target': 0.16646509407978655}, {'params': {'Kdv': 1.0, 'Kpv': 20.0, 'Kpr': 1.0, 'Kdr': 1.0}, 'target': 0.1536111932035037}, {'params': {'Kdv': 20.0, 'Kpv': 1.0, 'Kpr': 1.0, 'Kdr': 1.0}, 'target': 0.1041492010295832}, {'params': {'Kdv': 20.0, 'Kpv': 20.0, 'Kpr': 1.0, 'Kdr': 20.0}, 'target': 0.10849009278389787}, {'params': {'Kdv': 20.0, 'Kpv': 1.0, 'Kpr': 20.0, 'Kdr': 20.0}, 'target': 0.07186457267506281}, {'params': {'Kdv': 1.0, 'Kpv': 1.0, 'Kpr': 20.0, 'Kdr': 1.0}, 'target': 0.05444048894426966}, {'params': {'Kdv': 20.0, 'Kpv': 20.0, 'Kpr': 20.0, 'Kdr': 1.0}, 'target': 0.0819426168514994}, {'params': {'Kdv': 1.0, 'Kpv': 1.0, 'Kpr': 1.0, 'Kdr': 3.3539323601074624}, 'target': 0.21594482323946548}, {'params': {'Kdv': 1.0, 'Kpv': 11.49413405909773, 'Kpr': 1.0, 'Kdr': 11.778323998720422}, 'target': 0.1314492178143763}, {'params': {'Kdv': 20.0, 'Kpv': 1.0, 'Kpr': 1.0, 'Kdr': 20.0}, 'target': 0.17168447081965088}, {'params': {'Kdv': 9.359571423173737, 'Kpv': 1.0, 'Kpr': 8.435064617073389, 'Kdr': 11.522228821437032}, 'target': 0.28682243133293417}, {'params': {'Kdv': 12.041162776553964, 'Kpv': 12.34216673320052, 'Kpr': 11.147161981602979, 'Kdr': 9.677851955943822}, 'target': 0.2486154706651079}, {'params': {'Kdv': 10.55756360534677, 'Kpv': 4.9033797891521225, 'Kpr': 8.567917922917816, 'Kdr': 20.0}, 'target': 0.2005355182804937}, {'params': {'Kdv': 1.0, 'Kpv': 20.0, 'Kpr': 17.84841539278031, 'Kdr': 1.0}, 'target': 0.07130660934751741}, {'params': {'Kdv': 20.0, 'Kpv': 20.0, 'Kpr': 1.0, 'Kdr': 1.0}, 'target': 0.18216374428702417}, {'params': {'Kdv': 9.527431239335174, 'Kpv': 1.8655161890744334, 'Kpr': 7.464509017573529, 'Kdr': 3.8946476422128846}, 'target': 0.4064310598967727}, {'params': {'Kdv': 20.0, 'Kpv': 20.0, 'Kpr': 20.0, 'Kdr': 20.0}, 'target': 0.11057879210923777}, {'params': {'Kdv': 11.29040177368861, 'Kpv': 5.679673202445158, 'Kpr': 9.257034867795632, 'Kdr': 1.0}, 'target': 0.12910836515353724}]

    # {'params': {'Kdv': 1.068850210338851, 'Kpv': 6.999725044996498, 'Kpr': 7.21983127115623, 'Kdr': 1.0850709238210348}, 'target': 0.04491286165595498}
    # [{'params': {'Kdv': 14.686165375401004, 'Kpv': 6.744318880004956, 'Kpr': 1.0021731215295528, 'Kdr': 8.923418089348907}, 'target': 0.028998872926414017}, {'params': {'Kdv': 2.7544333006071584, 'Kpv': 7.5656538138179075, 'Kpr': 4.538944016175748, 'Kdr': 3.7883619255251477}, 'target': 0.044119593216902286}, {'params': {'Kdv': 1.0, 'Kpv': 20.0, 'Kpr': 20.0, 'Kdr': 20.0}, 'target': 0.014198077517518224}, {'params': {'Kdv': 20.0, 'Kpv': 1.0, 'Kpr': 20.0, 'Kdr': 1.0000000029713991}, 'target': 0.004095926170675027}, {'params': {'Kdv': 1.000000001797445, 'Kpv': 19.495753779734976, 'Kpr': 1.0, 'Kdr': 1.0}, 'target': 0.007277401589443593}, {'params': {'Kdv': 1.33895555853338, 'Kpv': 1.5823855500274855, 'Kpr': 2.721819341129623, 'Kdr': 19.931293411522134}, 'target': 0.04184792552106181}, {'params': {'Kdv': 1.1078104005355751, 'Kpv': 1.0339700845240936, 'Kpr': 1.9061047039722094, 'Kdr': 1.0978044300646321}, 'target': 0.0339762473828786}, {'params': {'Kdv': 1.0000000021820028, 'Kpv': 1.000000002686321, 'Kpr': 20.0, 'Kdr': 20.0}, 'target': 0.006830407665797228}, {'params': {'Kdv': 20.0, 'Kpv': 20.0, 'Kpr': 20.0, 'Kdr': 1.0000000007591958}, 'target': 0.004454053165384175}, {'params': {'Kdv': 1.2791359331667107, 'Kpv': 10.852962528695725, 'Kpr': 19.922772686103954, 'Kdr': 1.2341488605820392}, 'target': 0.003890610778724597}, {'params': {'Kdv': 19.95890896093425, 'Kpv': 19.725263031001973, 'Kpr': 12.564465296149434, 'Kdr': 19.849217054052225}, 'target': 0.01497794588955727}, {'params': {'Kdv': 1.0000000031518392, 'Kpv': 20.0, 'Kpr': 1.0000000138770377, 'Kdr': 20.0}, 'target': 0.010458070304290315}, {'params': {'Kdv': 7.426110121051638, 'Kpv': 1.0267157596856564, 'Kpr': 8.19416480790315, 'Kdr': 11.20021316053594}, 'target': 0.03047321413496085}, {'params': {'Kdv': 19.390287590196536, 'Kpv': 7.023138322548287, 'Kpr': 18.91539183802354, 'Kdr': 19.948936922933715}, 'target': 0.01024318966292522}, {'params': {'Kdv': 1.0418344165429787, 'Kpv': 5.542639452238831, 'Kpr': 1.1301288070491555, 'Kdr': 12.71816957846726}, 'target': 0.03357686537423664}, {'params': {'Kdv': 19.945459166950165, 'Kpv': 9.410715400855636, 'Kpr': 6.610367328405042, 'Kdr': 1.2092153059305106}, 'target': 0.0411103208521639}, {'params': {'Kdv': 6.546698722412592, 'Kpv': 9.925077875511242, 'Kpr': 10.15605722062294, 'Kdr': 19.94309326944354}, 'target': 0.02424964208949089}, {'params': {'Kdv': 19.99999998779669, 'Kpv': 1.0, 'Kpr': 1.0, 'Kdr': 19.999999993359392}, 'target': 0.027683201806856874}, {'params': {'Kdv': 18.590156817756473, 'Kpv': 1.2287180433725928, 'Kpr': 1.2816868441897347, 'Kdr': 1.2998575116251883}, 'target': 0.0330043394329916}, {'params': {'Kdv': 9.81135791651101, 'Kpv': 8.763955821102451, 'Kpr': 6.299733070223661, 'Kdr': 1.0920516804255485}, 'target': 0.04246427109246922}, {'params': {'Kdv': 1.068850210338851, 'Kpv': 6.999725044996498, 'Kpr': 7.21983127115623, 'Kdr': 1.0850709238210348}, 'target': 0.04491286165595498}, {'params': {'Kdv': 1.0686775117746459, 'Kpv': 9.115181182420288, 'Kpr': 7.4963211937367324, 'Kdr': 6.448061299186842}, 'target': 0.02948496895135204}]
