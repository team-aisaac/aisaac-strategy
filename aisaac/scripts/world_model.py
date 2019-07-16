#!/usr/bin/env  python
# coding:utf-8
import math
import numpy as npdr
import entity
import time, sys
import concurrent.futures
from sklearn.cluster import KMeans
import rospy
#from consai_msgs.msg import VisionPacket, VisionIDList
from consai_msgs.msg import robot_commands
from consai_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
from consai_msgs.msg import RefereeTeamInfo
from std_msgs.msg import Int8
import serial
#from aisaac.srv import Kick
from aisaac.msg import Status


from world_state import WorldState
from referee import Referee
from decision_maker import DecisionMaker
from objects import Objects
import functions

import world_model_utils

WORLD_LOOP_RATE = world_model_utils.WORLD_LOOP_RATE

#import physics as phy
"""
主にフィールドの情報の取得、物理的な衝突の確認をするコード
"""

class WorldModel():
    def __init__(self):
        rospy.init_node("world_model")
        self.team_color = str(rospy.get_param("team_color"))

        self.robot_total = 8
        self.enemy_total = 8

        """statusの立ち上げ msg"""
        self.status = [Status() for i in range(self.robot_total)]

        """statusのパラメータ初期化"""
        for id in range(self.robot_total):
            self.status[id].status = "None"
            self.status[id].pid_goal_pos_x = 0.
            self.status[id].pid_goal_pos_y = 0.
            self.status[id].pid_goal_theta = 0.
            self.status[id].pid_circle_center_x = 0.
            self.status[id].pid_circle_center_y = 0.
            self.status[id].pass_target_pos_x = 0.
            self.status[id].pass_target_pos_y = 0.

        #print(self.status)

        """----上の5つの変数、インスタンスをまとめたもの、callbackをもつ---"""
        self.objects = Objects(self.team_color, self.robot_total, self.enemy_total)

        """---World State(固定パラが多い)---"""
        self.world_state = WorldState(self.objects)

        """---Referee---"""
        self.referee = Referee(self.world_state)

        """---DecisionMaker---"""
        self.decision_maker = DecisionMaker(self.world_state, self.objects, self.referee, self.status)

        """---status Publisherオブジェクトのインスタンス---"""
        self.robot_0_status_pub = rospy.Publisher("/" + self.team_color + "/robot_0/status", Status, queue_size=10)
        self.robot_1_status_pub = rospy.Publisher("/" + self.team_color + "/robot_1/status", Status, queue_size=10)
        self.robot_2_status_pub = rospy.Publisher("/" + self.team_color + "/robot_2/status", Status, queue_size=10)
        self.robot_3_status_pub = rospy.Publisher("/" + self.team_color + "/robot_3/status", Status, queue_size=10)
        self.robot_4_status_pub = rospy.Publisher("/" + self.team_color + "/robot_4/status", Status, queue_size=10)
        self.robot_5_status_pub = rospy.Publisher("/" + self.team_color + "/robot_5/status", Status, queue_size=10)
        self.robot_6_status_pub = rospy.Publisher("/" + self.team_color + "/robot_6/status", Status, queue_size=10)
        self.robot_7_status_pub = rospy.Publisher("/" + self.team_color + "/robot_7/status", Status, queue_size=10)

    """---Refereeから司令をもらうsubscriberの起動--"""
    def referee_listener(self):
        """ rospy.Subscriber("refbox/command", Int8, self.referee.command_callback)
        rospy.Subscriber("refbox/stage", Int8, self.referee.stage_callback)
        rospy.Subscriber("refbox/blue_info", RefereeTeamInfo, self.referee.teaminfo_callback) """
        rospy.Subscriber("/" + self.team_color + "/refbox/command", Int8, self.referee.command_callback)
        rospy.Subscriber("/" + self.team_color + "/refbox/stage", Int8, self.referee.stage_callback)
        rospy.Subscriber("/" + self.team_color + "/refbox/blue_info", RefereeTeamInfo, self.referee.teaminfo_callback)

    """---robotのstatusのPubliber---"""
    def robot_status_publisher(self):
        self.robot_0_status_pub.publish(self.status[0])
        self.robot_1_status_pub.publish(self.status[1])
        self.robot_2_status_pub.publish(self.status[2])
        self.robot_3_status_pub.publish(self.status[3])
        self.robot_4_status_pub.publish(self.status[4])
        self.robot_5_status_pub.publish(self.status[5])
        self.robot_6_status_pub.publish(self.status[6])
        self.robot_7_status_pub.publish(self.status[7])


if __name__ == "__main__":
    a = WorldModel()
    # a.odom_listener()
    a.referee_listener()
    #a.objects.set_first_positions_4robots()
    #a.robot_status_publisher()
    loop_rate = rospy.Rate(WORLD_LOOP_RATE)
    print("start")

    assignment_x = [-4, -3, -2, -1, 0, 1, 2, 3]
    assignment_y = [1, 1, 1, 1, 1, 1, 1, 1]
    assignment_theta = [0, 0, 0, 0, 0, 0, 0, 0]
    # time.sleep(10)
    # a.decision_maker.goal_assignment(assignment_x, assignment_y, assignment_theta)

    try:
        while not rospy.is_shutdown():
            #print("loop")
            a.referee.referee_branch_decision()
            if a.referee.referee_branch == "HALT":
                print("HALT")
                a.decision_maker.stop_all()
            elif a.referee.referee_branch == "STOP":
                print("STOP")
                a.decision_maker.leave_from_ball()
            elif a.referee.referee_branch == "NORMAL_START":
                # ぼーるを落としてスタートする
                a.decision_maker.who_has_a_ball()
                a.decision_maker.update_strategy()
                print("START")
            elif a.referee.referee_branch == "KICKOFF":
                a.objects.set_first_position_4robots_attack()
                print("KICKOFF")
            elif a.referee.referee_branch == "DEFENCE":
                a.objects.set_first_positions_4robots()
                print("DEFENCE")

            #a.decision_maker.change_goal_status()
            # a.robot_status_publisher()
            loop_rate.sleep()
    except:
        print("error")
        a.stop_all()
