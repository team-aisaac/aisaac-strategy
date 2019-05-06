#!/usr/bin/env  python
# coding:utf-8
#hello world
import math
import numpy as np
import entity
import sys
import time
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
import struct
import serial
import time
#from aisaac.srv import Kick
from aisaac.msg import Status

WORLD_LOOP_RATE = 100.

#import physics as phy
"""
主にフィールドの情報の取得、物理的な衝突の確認をするコード
"""

class WorldModel():

    def __init__(self):
        rospy.init_node("world_model")

        self.team_color = str(rospy.get_param("team_color"))

        """---devisionの選択---"""
        self.devision = "A"
        self.devision = "B"

        """---味方ロボット台数と敵ロボット台数---"""
        self.robot_num = 4
        self.enemy_num = 8

        """---フィールドとロボットのパラメータ---"""
        self.robot_r = 0.09
        if self.devision == "A":
            self.field_x_size = 12.
            self.field_y_size = 9.
        elif self.devision == "B":
            self.field_x_size = 9.
            self.field_y_size = 6.
        self.field_x_min = - self.field_x_size / 2.
        self.field_x_max = self.field_x_size / 2.
        self.field_y_min = - self.field_y_size / 2.
        self.field_y_max = self.field_y_size / 2.
        self.goal_y_size = 1.2
        self.goal_y_min = -self.goal_y_size / 2.
        self.goal_y_max = self.goal_y_size / 2.

        """---team cloorと攻撃方向---"""
        #self.color = "Yellow"
        self.color = "Blue"
        self.goal_direction = "Right"
        #self.goal_direction = "Left"

        """---PID制御用パラメータ---"""
        self.Kpv = 0.3
        self.Kpr = 0.1

        """---Potential法用パラメータ---"""
        self.Kpv_2 = 0.5
        self.Kpr_2 = 0.1

        """---refereeからの指示をもとにどのループを回すかの指標---"""
        self.referee_branch = None

        """---ボール軌道の考慮時間幅(linear Regressionで軌道予測するため)---"""
        self.ball_dynamics_window = 5
        self.ball_dynamics_x = [0. for i in range(self.ball_dynamics_window)]
        self.ball_dynamics_y = [0. for i in range(self.ball_dynamics_window)]

        self.goal_keeper_x = self.field_x_min + 0.05

        """---robot、ballオブジェクトのインスタンス---"""
        self.robot = [entity.Robot() for i in range(self.robot_num)]
        self.enemy = [entity.Robot() for i in range(self.enemy_num)]
        self.ball = entity.Ball()


        """---Publisherオブジェクトのインスタンス---"""
        self.robot_0_pub = rospy.Publisher("/" + self.team_color + "/robot_0/robot_commands", robot_commands, queue_size=10)
        self.robot_1_pub = rospy.Publisher("/" + self.team_color + "/robot_1/robot_commands", robot_commands, queue_size=10)
        self.robot_2_pub = rospy.Publisher("/" + self.team_color + "/robot_2/robot_commands", robot_commands, queue_size=10)
        self.robot_3_pub = rospy.Publisher("/" + self.team_color + "/robot_3/robot_commands", robot_commands, queue_size=10)

        """
        self.robot_4_pub = rospy.Publisher("/robot_4/robot_commands", robot_commands, queue_size=10)
        self.robot_5_pub = rospy.Publisher("/robot_5/robot_commands", robot_commands, queue_size=10)
        self.robot_6_pub = rospy.Publisher("/robot_6/robot_commands", robot_commands, queue_size=10)
        self.robot_7_pub = rospy.Publisher("/robot_7/robot_commands", robot_commands, queue_size=10)
        """

        """---status Publisherオブジェクトのインスタンス---"""
        self.robot_0_status_pub = rospy.Publisher("/" + self.team_color + "/robot_0/status", Status, queue_size=10)
        self.robot_1_status_pub = rospy.Publisher("/" + self.team_color + "/robot_1/status", Status, queue_size=10)
        self.robot_2_status_pub = rospy.Publisher("/" + self.team_color + "/robot_2/status", Status, queue_size=10)
        self.robot_3_status_pub = rospy.Publisher("/" + self.team_color + "/robot_3/status", Status, queue_size=10)
        self.robot_4_status_pub = rospy.Publisher("/" + self.team_color + "/robot_4/status", Status, queue_size=10)
        self.robot_5_status_pub = rospy.Publisher("/" + self.team_color + "/robot_5/status", Status, queue_size=10)
        self.robot_6_status_pub = rospy.Publisher("/" + self.team_color + "/robot_6/status", Status, queue_size=10)
        self.robot_7_status_pub = rospy.Publisher("/" + self.team_color + "/robot_7/status", Status, queue_size=10)

        """pid用のflag初期化"""
        for id in range(self.robot_num):
            self.robot[id].pid_init_flag = True

        """cmdの立ち上げ"""
        self.cmd = [robot_commands() for i in range(self.robot_num)]

        """statusの立ち上げ"""
        self.status = [Status() for i in range(self.robot_num)]

        """cmdのパラメータ初期化"""
        for id in range(self.robot_num):
            self.cmd[id].vel_surge = 0. #vel_x
            self.cmd[id].vel_sway = 0.  #vel_y
            self.cmd[id].omega = 0.     #vel_theta
            self.cmd[id].kick_speed_x = 0
            self.cmd[id].kick_speed_z = 0
            self.cmd[id].dribble_power = 0

        """statusのパラメータ初期化"""
        for id in range(self.robot_num):
            self.status[id].status = "None"


        """positionの割り振り(動的にPositionが切り替わるのは多分大事、現状使っていない)"""
        """
        if self.robot_num == 8:
            self.robot[0].position = "GK"
            self.robot[1].position = "LCB"
            self.robot[2].position = "RCB"
            self.robot[3].position = "LSB"
            self.robot[4].position = "RSB"
            self.robot[5].position = "LMF"
            self.robot[6].position = "RMF"
        elif self.robot_num == 4:
            self.robot[0].position = "GK"
            self.robot[1].position = "LCB"
            self.robot[2].position = "CCB"
            self.robot[3].position = "RCB"
        """

        """publishのrate(多分rospyのを使うからいらない)"""
        self.publish_rate = 0.05 #s #

        """strategyの選択(いまはつかっていない)"""
        self.strategy = None

        """refereeから受信する情報(現状cmdしか使っていない)"""
        self.referee = None

        """---敵と味方どっちがボールをもっているか(現状使ってない)---"""
        self.which_has_a_ball = None

        """---攻撃かdefenceか(これをうまく使っていきたい)---"""
        self.attack_or_defence = None

        """---ポテンシャル法用のパラメータ（1/x）---"""
        self.weight_enemy = 1.0
        self.weight_goal = 100.0
        self.delta = 0.0000001

        """---ポテンシャル法用のパラメータ（ガウス関数)---"""
        self.weight_enemy_2 = 7.0
        self.weight_goal_2 = 20000000.0
        self.width_enemy = 0.1
        self.width_goal = 0.1
        self.delta_2 = 0.00001

    def set_init_positions(self):
        self.robot[0].set_future_position(x=-6., y=0., theta=0.)
        self.robot[1].set_future_position(x=-4.5, y=1., theta=0.)
        self.robot[2].set_future_position(x=-4.5, y=-1., theta=0.)
        self.robot[3].set_future_position(x=-3., y=3.5, theta=0.)
        self.robot[4].set_future_position(x=-3., y=-3.5, theta=0.)
        self.robot[5].set_future_position(x=-1., y=2., theta=0.)
        self.robot[6].set_future_position(x=-1., y=-2., theta=0.)
        self.robot[7].set_future_position(x=-0.5, y=0., theta=0.)

    def set_first_positions(self):
        self.robot[0].set_future_position(x=-5.5, y=0., theta=0.)
        self.robot[1].set_future_position(x=-4.5, y=1., theta=0.)
        self.robot[2].set_future_position(x=-4.5, y=-1., theta=0.)
        self.robot[3].set_future_position(x=-2., y=2.5, theta=0.)
        self.robot[4].set_future_position(x=-2., y=-2.5, theta=0.)
        self.robot[5].set_future_position(x=1.5, y=2.5, theta=0.)
        self.robot[6].set_future_position(x=1.5, y=-2.5, theta=0.)
        self.robot[7].set_future_position(x=2.5, y=0., theta=0.)

    """---4台用守備時(20181216練習会用に作られた)---"""
    def set_first_positions_4robots(self):
        self.robot[0].set_future_position(x=-5.5, y=0., theta=0.)
        self.robot[1].set_future_position(x=-4.5, y=1., theta=0.)
        self.robot[2].set_future_position(x=-4.5, y=-1., theta=0.)
        self.robot[3].set_future_position(x=-2., y=0., theta=0.)

    """---4台用守備時(20181216練習会用に作られた)---"""
    def set_first_position_4robots_attack(self):
        self.robot[0].set_future_position(x=-5.5, y=0., theta=0.)
        self.robot[1].set_future_position(x=-4.5, y=1., theta=0.)
        self.robot[2].set_future_position(x=-4.5, y=-1., theta=0.)
        self.robot[3].set_future_position(x=0.-self.robot_r, y=0., theta=0.)

    """---matplot上に描画する(多分いらない)---"""
    def show_positions_figure(self):
        #graph = figure.PositionFigure()
        for robot in self.robot:
            x, y, theta = robot.get_current_position()
            #graph.set_robot_plot(x, y, z)
            figure = plt.plot(x,y,"ro")
            print(x,y,theta)
            plt.setp(figure, markersize=10)

        for enemy in self.enemy:
            x, y, theta = enemy.get_current_position()
            #graph.set_robot_plot(x, y, z)
            figure = plt.plot(x,y,"bo")
            print(x,y,theta)
            plt.setp(figure, markersize=10)

        plt.xlim(-self.field_x_size/2, self.field_x_size/2)
        plt.ylim(-self.field_y_size/2, self.field_y_size/2)
        plt.show()

    """---衝突だけを判定---"""
    def robot_collision_detect(self):
        robot = self.robot + self.enemy
        for i in range(self.robot_num*2):
            position_1_x, position_1_y, z = robot[i].get_current_position()
            for j in range(self.robot_num*2-i-1):
                position_2_x, position_2_y, z = robot[j+i+1].get_current_position()
                if (position_1_x - position_2_x)**2 + (position_1_y - position_2_y)**2 < 0.18*0.18:
                    print("dangerous!!")
                    #sys.exit()
                    break
            else:
                continue
            break
        else:
            print("safety!!")

    """---衝突を判定して避ける動きをする---"""
    def robot_collision_detect_2(self):
        safety_rate = 1.1
        robot = self.robot + self.enemy
        for i in range(self.robot_num):
            position_1_x, position_1_y, _ = robot[i].get_current_position()
            for j in range(self.robot_num*2-i-1):
                position_2_x, position_2_y, _ = robot[j+i+1].get_current_position()
                if (position_1_x - position_2_x)**2 + (position_1_y - position_2_y)**2 < (self.robot_r * 2 * 1.1)**2:
                    if j < self.robot_num:
                        self.cmd[i].vel_surge = -1. * self.cmd[i].vel_surge
                        self.cmd[i].vel_sway = -1. * self.cmd[i].vel_sway
                        self.cmd[i].omega = 0.     #vel_theta
                    else:
                        self.cmd[i].vel_surge = 0.
                        self.cmd[i].vel_sway = 0.
                        self.cmd[i].omega = 0.     #vel_theta


    """フィールドをメッシュ化する(クラスタリング用に書かれた)"""
    def create_mesh(self):
        self.mesh_size = 50
        self.mesh_x_num = int(self.field_x_size / (2*self.mesh_size)) + 1
        self.mesh_y_num = int(self.field_y_size / self.mesh_size) + 1
        self.mesh_x = [ i * self.mesh_size for i in range(self.mesh_x_num)]
        self.mesh_y = [- self.field_y_size / 2 + i * self.mesh_size for i in range(self.mesh_y_num)]
        #print(self.mesh_x)
        #print(self.mesh_y)
        self.mesh_xy = []
        for x in self.mesh_x:
            for y in self.mesh_y:
                self.mesh_xy.append([x, y])
        return self.mesh_xy

    """各メッシュの密度計算"""
    def enemy_density(self):
        mesh_xy = np.array(self.create_mesh())
        distance = np.zeros([self.mesh_x_num * self.mesh_y_num])
        enemy_position = []
        for i in range(self.robot_num-1):
            x, y, _ = self.enemy[i+1].get_current_position()
            #print(x,y)
            distance_ = (mesh_xy - np.array([x,y]))**2
            distance += np.sqrt(distance_[:,0] + distance_[:,1])
        mean_distance = distance / 7
        return mean_distance.reshape([self.mesh_x_num, self.mesh_y_num])

    """密度からk-meansでクラス境界を獲得"""
    def space_clustering(self):
        self.threshold = 3000
        self.n_cluster = 3
        mean_distance = self.enemy_density().reshape([self.mesh_x_num * self.mesh_y_num])
        mesh_xy = np.array(self.mesh_xy)
        plots = mesh_xy[mean_distance > self.threshold]
        print(plots.shape)
        cls = KMeans(self.n_cluster)
        pred = cls.fit_predict(plots)

        #"""
        for i in range(self.n_cluster):
            labels = plots[pred == i]
            plt.scatter(labels[:, 0], labels[:, 1])
        #"""
        centers = cls.cluster_centers_

        plt.scatter(centers[:, 0], centers[:, 1], s=100,
                facecolors='none', edgecolors='black')
        plt.xlim(-self.field_x_size/2, self.field_x_size/2)
        plt.ylim(-self.field_y_size/2, self.field_y_size/2)
        plt.show()


    """誰がボールを持っているかの判定"""
    def who_has_a_ball(self):
        x_ball, y_ball, _ = self.ball.get_current_position()
        flag = 0
        area = 0.015
        for i in range(self.robot_num):
            x_robot, y_robot, _ = self.robot[i].get_current_position()
            if (x_ball - x_robot)**2 + (y_ball - y_robot)**2 < (self.robot_r + area)**2:
                self.robot[i].has_a_ball = True
                #print("check:",i)
                flag = flag + 1
            else:
                self.robot[i].has_a_ball = False
        for i in range(self.robot_num):
            x_robot, y_robot, _ = self.enemy[i].get_current_position()
            if (x_ball - x_robot)**2 + (y_ball - y_robot)**2 < (self.robot_r + area)**2:
                self.enemy[i].has_a_ball = True
                flag = flag - 1
            else:
                self.enemy[i].has_a_ball = False
        if flag == 1:
            self.which_has_a_ball = "robots"
        elif flag == -1:
            self.which_has_a_ball = "enemy"
        else:
            self.which_has_a_ball = "free"

    """---攻撃、守備の選択(現状はボールを持っているかだけで決まっている)---"""
    def attack_or_defence(self):
        if self.which_has_a_ball == "robots":
            self.attack_or_defence = "attack"
        else:
            self.attack_or_defence = "defence"

    """---攻撃の戦略の選択(現状使えていない)---"""
    def decide_attack_strategy(self):
        if self.attack_or_defence == "attack":
            #self.space_clustering(
            print("attack!")
        else:
            print("defence!")
            #None

    """---ポテンシャルの計算(1/x型)---"""
    def get_potential(self, id, x, y, x_goal, y_goal):
        U = 0.
        for i in range(self.robot_num):
            if i != id:
                x_robot, y_robot, _ = self.robot[i].get_current_position()
                U +=  (self.weight_enemy *  1.0) / math.sqrt((x - x_robot + 0.00001)*(x - x_robot + 0.00001) + (y - y_robot + 0.00001)*(y - y_robot + 0.00001))
        for j in range(self.enemy_num):
            x_enemy, y_enemy, _ = self.enemy[j].get_current_position()
            U +=  (self.weight_enemy *  1.0) / math.sqrt((x - x_enemy + 0.00001)*(x - x_enemy + 0.00001) + (y - y_enemy + 0.00001)*(y - y_enemy + 0.00001))
        U += (self.weight_goal * -1.0) / math.sqrt((x - x_goal + 0.00001)*(x - x_goal + 0.00001) + (y - y_goal + 0.00001)*(y - y_goal + 0.00001));
        return U

    """---ポテンシャルの計算(ガウス関数型)、味方のポテンシャル入ってない---"""
    def get_potential_2(self, x, y, x_goal, y_goal):
        U = 0.
        for i in range(self.enemy_num):
            x_enemy, y_enemy, _ = self.enemy[i].get_current_position()
            U +=  self.weight_enemy_2 * np.exp(-1 * ( (x - x_enemy) **2 + (y - y_enemy) **2 ) / self.width_enemy)
        U += self.weight_goal_2 * (1 - np.exp(-1 * ( (x - x_goal)**2 + (y - y_goal)**2) / self.width_goal) )
        return U

    """---全機体ポテンシャル法で移動---"""
    def move_by_potential_method_all(self):
        for id in range(self.robot_num):
            self.mive_by_potential_method_single(id)

    """---各機体ポテンシャル法で移動---"""
    def move_by_potential_method_single(self, id):
        x_goal, y_goal, theta_goal = self.robot[id].get_future_position()
        x_current, y_current, theta_current = self.robot[id].get_current_position()
        vx = - (self.get_potential(id, x_current + self.delta, y_current, x_goal, y_goal) - self.get_potential(id, x_current, y_current, x_goal, y_goal)) / self.delta
        vy = - (self.get_potential(id, x_current, y_current + self.delta, x_goal, y_goal) - self.get_potential(id, x_current, y_current, x_goal, y_goal)) / self.delta
        v = math.sqrt(vx * vx + vy * vy)
        distance = np.sqrt( (x_goal - x_current)**2 + (y_goal - y_current)**2 )
        Vx = vx * self.Kpv_2 * distance / v
        Vy = vy * self.Kpv_2 * distance / v
        Vr = self.Kpr * (theta_goal - theta_current)
        self.cmd[id].vel_surge, self.cmd[id].vel_sway = self.velocity_transformation(id, Vx, Vy)
        self.cmd[id].omega = Vr   #vel_theta

    """---全機体ポテンシャル法で移動(ガウス)---"""
    def move_by_potential_method_2(self):
        for id in range(self.robot_num):
            x_goal, y_goal, theta_goal = self.robot[id].get_future_position()
            x_current, y_current, theta_current = self.robot[id].get_current_position()
            vx = - (self.get_potential_2(x_current + self.delta_2, y_current, x_goal, y_goal) - self.get_potential_2(x_current, y_current, x_goal, y_goal)) / self.delta_2
            vy = - (self.get_potential_2(x_current, y_current + self.delta_2, x_goal, y_goal) - self.get_potential_2(x_current, y_current, x_goal, y_goal)) / self.delta_2
            v = math.sqrt(vx * vx + vy * vy)
            distance = np.sqrt( (x_goal - x_current)**2 + (y_goal - y_current)**2 )
            Vx = vx * self.Kpv_2 * distance / v
            Vy = vy * self.Kpv_2 * distance / v
            Vr = self.Kpr * (theta_goal - theta_current)
            self.cmd[id].vel_surge, self.cmd[id].vel_sway = self.velocity_transformation(id, Vx, Vy)
            self.cmd[id].omega = Vr    #vel_theta

    """---ボールの軌道に直線をフィッティング y=ax+bのaとbを返す---"""
    def ball_liner_fitting(self):
        while len(self.ball_dynamics_x) != self.ball_dynamics_window:
            if len(self.ball_dynamics_x) < self.ball_dynamics_window:
                self.ball_dynamics_x.append(0.)
            else:
                _ = self.ball_dynamics_x.pop(0)

        while len(self.ball_dynamics_y) != self.ball_dynamics_window:
            if len(self.ball_dynamics_y) < self.ball_dynamics_window:
                self.ball_dynamics_y.append(0.)
            else:
                _ = self.ball_dynamics_y.pop(0)

        array_x = np.array(self.ball_dynamics_x)
        array_y = np.array(self.ball_dynamics_y)
        n = self.ball_dynamics_window
        xy_sum = np.dot(array_x, array_y)
        x_sum = np.sum(array_x)
        y_sum = np.sum(array_y)
        x_square_sum = np.dot(array_x, array_x)
        a = (n * xy_sum - x_sum * y_sum) / (n * x_square_sum - (x_sum ** 2) + 0.00001)
        b = (x_square_sum * y_sum - xy_sum * x_sum) / (n * x_square_sum - x_sum ** 2 + 0.00001)
        return a,b

    """---GKは常に直線フィッティングの先へと移動する---"""
    def goal_keeper_strategy(self):
        a, b = self.ball_liner_fitting()
        y = a * self.goal_keeper_x + b
        if y > self.goal_y_min and y < self.goal_y_max:
            self.robot[0].set_future_position(self.goal_keeper_x, y, 0.)

    """---戦略を決定する部分(現状はつかっていない)---"""
    def decision_making(self):
        while True:
            self.who_has_a_ball()
            self.attack_or_defence()
            self.decide_attack_strategy()
            if self.attack_or_defence == "attack":
                None
            elif self.attack_or_defence == "defence":
                self.defence_strategy()
            print("decision_making")
            time.sleep(0.1)

    """---Visionからrobotの現在地をもらう---"""
    def robot_odom_callback(self, msg, id):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.robot[id].set_current_position(x = robot_x, y = robot_y, theta=robot_t[2])

    """---Visionからenemyの現在地をもらう---"""
    def enemy_odom_callback(self, msg, id):
        enemy_x = msg.pose.pose.position.x
        enemy_y = msg.pose.pose.position.y
        enemy_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.enemy[id].set_current_position(x = enemy_x, y = enemy_y, theta=enemy_t[2])

    """---Visionからballの現在地をもらう---"""
    def ball_odom_callback(self, msg):
        ball_x = msg.pose.pose.position.x
        ball_y = msg.pose.pose.position.y
        ball_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.ball.set_current_position(x = ball_x, y = ball_y, theta=ball_t[2])
        _ = self.ball_dynamics_x.pop(0)
        _ = self.ball_dynamics_x.pop(0)
        self.ball_dynamics_x.append(ball_x)
        self.ball_dynamics_x.append(ball_y)

    """---Refereeから現在のcommandをもらう---"""
    def ref_command_callback(self, msg):
        self.referee = str(msg)
        #print(self.referee)

    """---Refereeから現在のstageをもらう---"""
    def ref_stage_callback(self, msg):
        None
        #self.referee = msg
        #print(msg)

    """---Refereeから現在のteaminfoをもらう---"""
    def ref_teaminfo_callback(self, msg):
        None
        #print("-----------------teaminfo-------------------")
        #print(msg)
        #print("--------------------------------------------")

    """---Visionから現在地をもらうsubscriberの起動--"""
    def odom_listener(self):
        for i in range(self.robot_num):
            rospy.Subscriber("/" + self.team_color + "/robot_"+ str(i) +"/odom", Odometry, self.robot_odom_callback, callback_args=i)

        for j in range(self.enemy_num):
            rospy.Subscriber("/" + self.team_color + "/enemy_" + str(j) + "/odom", Odometry, self.enemy_odom_callback, callback_args=j)

        rospy.Subscriber("/" + self.team_color + "/ball_observer/estimation", Odometry, self.ball_odom_callback)
        #rospy.spin()

    """---Refereeから司令をもらうsubscriberの起動--"""
    def referee_listener(self):
        rospy.Subscriber("/refbox/command", Int8, self.ref_command_callback)
        rospy.Subscriber("/refbox/stage", Int8, self.ref_stage_callback)
        rospy.Subscriber("/refbox/" + self.team_color + "_info", RefereeTeamInfo, self.ref_teaminfo_callback)

    """---各機体にベクトルをPublishする--"""
    def robot_cmd_publisher_2(self):
        self.robot_0_pub.publish(self.cmd[0])
        self.robot_1_pub.publish(self.cmd[1])
        self.robot_2_pub.publish(self.cmd[2])
        self.robot_3_pub.publish(self.cmd[3])
        self.robot_4_pub.publish(self.cmd[4])
        self.robot_5_pub.publish(self.cmd[5])
        self.robot_6_pub.publish(self.cmd[6])
        self.robot_7_pub.publish(self.cmd[7])
        time.sleep(self.publish_rate)

    """---各機体にベクトルをPublishする（4台ver）---"""
    def robot_cmd_publisher_4robots(self):
        #print("publisher")
        self.robot_0_pub.publish(self.cmd[0])
        self.robot_1_pub.publish(self.cmd[1])
        self.robot_2_pub.publish(self.cmd[2])
        self.robot_3_pub.publish(self.cmd[3])
        """
        for id in range(robot_num):
            self.cmd_to_byte(id)
        """
        #time.sleep(self.publish_rate)

    """---各機体にベクトルをPublishする---"""
    def robot_cmd_publisher(self, id):
        #print("publisher")
        self.robot_0_pub.publish(self.cmd[0])
        self.robot_1_pub.publish(self.cmd[1])
        self.robot_2_pub.publish(self.cmd[2])
        self.robot_3_pub.publish(self.cmd[3])
        self.robot_4_pub.publish(self.cmd[4])
        self.robot_5_pub.publish(self.cmd[5])
        self.robot_6_pub.publish(self.cmd[6])
        self.robot_7_pub.publish(self.cmd[7])
        time.sleep(self.publish_rate)

    def robot_status_publisher(self):
        self.robot_0_status_pub.publish(self.status[0])
        """
        self.robot_1_status_pub.publish(self.status[1])
        self.robot_2_status_pub.publish(self.status[2])
        self.robot_3_status_pub.publish(self.status[3])
        self.robot_4_status_pub.publish(self.status[4])
        self.robot_5_status_pub.publish(self.status[5])
        self.robot_6_status_pub.publish(self.status[6])
        self.robot_7_status_pub.publish(self.status[7])
        """

    """---ベクトルをWorld座標ヵらRobot座標へ---"""
    def velocity_transformation(self, id, vx, vy):
        theta = self.robot[id].current_orientation
        v_surge = vx * np.cos(theta) + vy * np.sin(theta) * (-1)
        v_sway = vx * np.sin(theta) + vy * np.cos(theta)
        return v_surge, v_sway

    """---P制御で全機体動かす(PDができたからいらない)---"""
    def pid_all(self):
        for id in range(self.robot_num):
            goal_x, goal_y, goal_theta = self.robot[id].get_future_position()
            current_position_x, current_position_y, current_orientation = self.robot[id].get_current_position()
            d_x = goal_x - current_position_x
            d_y = goal_y - current_position_y
            d_theta = goal_theta - current_orientation
            #distance = math.sqrt(d_x * d_x + d_y * d_y)
            Vx =  self.Kpv * d_x
            Vy =  self.Kpv * d_y
            Vr = self.Kpr * d_theta
            self.cmd[id].vel_surge, self.cmd[id].vel_sway = self.velocity_transformation(id, Vx, Vy)
            self.cmd[id].omega = Vr     #vel_theta

    """---P制御で各機体動かす(PDができたからいらない)---"""
    def pid_single(self, id):
        goal_x, goal_y, goal_theta = self.robot[id].get_future_position()
        current_position_x, current_position_y, current_orientation = self.robot[id].get_current_position()
        d_x = goal_x - current_position_x
        d_y = goal_y - current_position_y
        d_theta = goal_theta - current_orientation
        #distance = math.sqrt(d_x * d_x + d_y * d_y)
        Vx =  self.Kpv * d_x
        Vy =  self.Kpv * d_y
        Vr = self.Kpr * d_theta
        self.cmd[id].vel_surge, self.cmd[id].vel_sway = self.velocity_transformation(id, Vx, Vy)
        self.cmd[id].omega = Vr    #vel_theta

    """---点と直線の距離の計算---"""
    def distance_of_a_point_and_a_straight_line(self, x_0, y_0, a, b, c):
        d = abs(a * x_0 + b * y_0 + c) / np.sqrt(a**2 + b**2)
        return d
      
    """
    def calculate_all_distance(self, goal_pos_x, goal_pos_y):
        for i in range(len(self.robot_num)):
            pos_x, pos_y = self.robot[i].get_current_position()
            for j in range(len(goal_pos_x)):
                distance[i][j] = np.sqrt( (goal_pos_x[j] - pos_x)**2 + (goal_pos_y[j] - pos_y)**2 )
        return distance[i][j]

    """

    
    def count_collision(self, robot_id, current_pos_x, current_pos_y, goal_pos_x, goal_pos_y):
        counter = 0
        a, b, c = self.line_parameters(current_pos_x, current_pos_y, goal_pos_x, goal_pos_y)
        if a != 0 and b != 0:
            for i in range(self.robot_num):
                if i != robot_id:
                    friend_pos_x, friend_pos_y, _ = self.robot[i].get_current_position()
                    distance = self.distance_of_a_point_and_a_straight_line(friend_pos_x, friend_pos_y, a, b, c)
                    if distance < self.robot_r * 3:
                        x = (-friend_pos_y * b + (b**2 / a) * friend_pos_x - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (current_pos_x < x < goal_pos_x or current_pos_x > x > goal_pos_x) and (current_pos_y < y < goal_pos_y or current_pos_y > y > goal_pos_y):
                            counter += 1
            for j in range(self.enemy_num):
                enemy_pos_x, enemy_pos_y, _ = self.enemy[j].get_current_position()
                distance = self.distance_of_a_point_and_a_straight_line(enemy_pos_x, enemy_pos_y, a, b, c)
                if distance < self.robot_r * 3:
                    x = (-enemy_pos_y * b + (b**2 / a) * enemy_pos_x - c) / (a + b**2 / a)
                    y = (-a * x -c) / b
                    if (current_pos_x < x < goal_pos_x or current_pos_x > x > goal_pos_x) and (current_pos_y < y < goal_pos_y or current_pos_y > y > goal_pos_y):
                        counter += 1

            ball_pos_x, ball_pos_y, _ = self.ball.get_current_position()
            distance = self.distance_of_a_point_and_a_straight_line(ball_pos_x, ball_pos_y, a, b, c)
            if distance < self.robot_r * 2:
                x = (-ball_pos_y * b + (b**2 / a) * ball_pos_x - c) / (a + b**2 / a)
                y = (-a * x -c) / b
                if (current_pos_x < x < goal_pos_x or current_pos_x > x > goal_pos_x) and (current_pos_y < y < goal_pos_y or current_pos_y > y > goal_pos_y):
                    counter += 1
        return counter

    def calculate_move_cost(self, robot_id, goal_x, goal_y):
        current_x, current_y, _ = self.robot[robot_id].get_current_position()
        current_vx, current_vy, _ = self.robot[robot_id].get_current_velocity()

        distance = np.sqrt( (goal_x - current_x)**2 + (goal_y - current_y)**2 )
        #velocity_difference = np.sqrt( (((goal_x - current_x) * 3.7 / distance) - current_vx)**2 + (((goal_y - current_y) * 3.7 / distance) - current_vy)**2 )
        collision = self.count_collision(robot_id, current_x, current_y, goal_x, goal_y)
        velocity_difference = 0
        
        if collision == 0:
            move_cost = distance * 1 + velocity_difference * 0.1
        else:
            move_cost = distance * 1 + velocity_difference * 0.1 + (1 / collision) * 3

        return move_cost

    
    def goal_assignment(self, assignment_x, assignment_y, assignment_theta):
        best_cost = 100
        best_id = 0
        used_id = []
        for  priority in range(len(assignment_x)):
            for robot_id in range(self.robot_num):
                current_cost = self.calculate_move_cost(robot_id, assignment_x[priority], assignment_y[priority])
                if (best_cost > current_cost) and (str(robot_id) not in used_id):
                    best_cost = current_cost
                    best_id = robot_id
            used_id.append(str(best_id))
            best_cost = 100
            self.status[best_id].pid_goal_pos_x = assignment_x[priority]
            self.status[best_id].pid_goal_pos_y = assignment_y[priority]
            self.status[best_id].pid_goal_theta = assignment_theta[priority]
            self.status[best_id].status = "move_linear"
            print best_id
            print self.status[best_id].pid_goal_pos_x
            print self.status[best_id].pid_goal_pos_y
            print self.status[best_id].pid_goal_theta

    """---2点をつなぐ直線ax+by+cのa,b,cを返す---"""
    def line_parameters(self, x_1, y_1, x_2, y_2):
        a = y_1 - y_2
        b = x_2 - x_1
        c = x_1 * y_2 - x_2 * y_1
        return a, b, c

    """---目的地までにてきがいたらポテンシャル、いなければ直線(現状使っていない)---"""
    def move_pd(self):
        for id in range(self.robot_num):
            counter = 0
            x_1, y_1, _ = self.robot[id].get_current_position()
            x_2, y_2, _ = self.robot[id].get_future_position()
            a, b, c = self.line_parameters(x_1, y_1, x_2, y_2)
            for id_2 in range(self.robot_num):
                if id != id_2:
                    x_0, y_0, _ = self.robot[id_2].get_current_position()
                    d = self.distance_of_a_point_and_a_straight_line(x_0, y_0, a, b, c)
                    if d < self.robot_r:
                        counter += 1
            for id_3 in range(self.enemy_num):
                x_0, y_0, _ = self.enemy[id_3].get_current_position()
                d = self.distance_of_a_point_and_a_straight_line(x_0, y_0, a, b, c)
                if d < self.robot_r:
                    counter += 1
            if counter == 0:
                self.pid_single(id)
            else:
                self.move_by_potential_method_single(id)

    """---20181216時の戦略---"""
    def update_strategy(self):
        robot_x, robot_y, _ = self.robot[3].get_current_position()
        ball_x, ball_y, _ = self.ball.get_current_position()
        close_range = 0.3
        if robot_x**2 + robot_y**2 < (self.robot_r + close_range) **2:
            theta = np.arctan( (0. - robot_y) / (self.field_x_max - robot_x) )
            self.robot[3].set_future_position(ball_x, ball_y, theta)
        else:
            self.check_enemy_position()

            ball_flag = ""
            if ball_x < 0.:
                robot_x, robot_y, _ = self.robot[3].get_current_position()
                theta = np.arctan( (robot_y - ball_y) / (robot_x - ball_x) )
                self.robot[3].set_future_position(ball_x, ball_y, theta)
                if ball_x <-3. and ball_y > 0.:
                    ball_flag = "area1"
                elif ball_x <-3. and ball_y < 0.:
                    ball_flag = "area2"
                elif ball_y > 0.:
                    ball_flag = "area3"
                else:
                    ball_flag = "area4"
            else:
                self.robot[3].set_future_position(-2., 0., 0.)

            for data in self.enemy_info:
                if ball_flag == "area1" or ball_flag == "area3":
                    if data[1] == "area1" or data[1] == "area3":
                        #print("ok1")
                        enemy_x, enemy_y, _ = self.enemy[data[0]].get_current_position()
                        robot_x, robot_y ,_ = self.robot[1].get_current_position()
                        a, b, c = self.line_parameters(x_1=-6., y_1=0., x_2=enemy_x, y_2=enemy_y)
                        target_x, target_y = self.calculate_contact(a, b, c, robot_x, robot_y)
                        theta = np.arctan(- a / b)
                        self.robot[1].set_future_position(target_x, target_y, theta)
                        self.robot[2].set_future_position(-4., -1.5, 0.)
                elif ball_flag == "area2" or ball_flag == "area4":
                    if data[1] == "area2" or data[1] == "area4":
                        #print("ok2")
                        enemy_x, enemy_y, _ = self.enemy[data[0]].get_current_position()
                        robot_x, robot_y ,_ = self.robot[2].get_current_position()
                        a, b, c = self.line_parameters(x_1=-6., y_1=0., x_2=enemy_x, y_2=enemy_y)
                        target_x, target_y = self.calculate_contact(a, b, c, robot_x, robot_y)
                        theta = np.arctan(- a / b)
                        self.robot[2].set_future_position(target_x, target_y, theta)
                        self.robot[1].set_future_position(-4., 1.5, 0.)
                else:
                    self.robot[1].set_future_position(-4., 1.5, 0.)
                    self.robot[2].set_future_position(-4., -1.5, 0.)
            self.goal_keeper_strategy()


    """---点から直線に推薦を引いた時の接点(x, y)の計算---"""
    def calculate_contact(self, a, b, c, x_0, y_0):
        denominator = 1 / (a**2 + b**2)
        x = denominator * (b**2 * x_0 - a * b * y_0 - a * c)
        y = denominator * (-a * b * x_0 + a**2 * y_0 - b * c)
        return x, y

    """---敵のPositionを確認する---"""
    def check_enemy_position(self):
        self.enemy_info = []
        for id in range(self.enemy_num):
            enemy_x, enemy_y, _ = self.enemy[id].get_current_position()
            if enemy_x < -3.:
                if enemy_y > 0.:
                    self.enemy_info.append([id, "area1"])
                else:
                    self.enemy_info.append([id, "area2"])
            elif enemy_x < 0.:
                if enemy_y > 0.:
                    self.enemy_info.append([id, "area3"])
                else:
                    self.enemy_info.append([id, "area4"])

    """---自分の近くにボールがあるときキックする---"""
    def kick(self):
        for id in range(self.robot_num):
            if self.robot[id].has_a_ball == True:
                #print("check")
                self.cmd[id].kick_speed_x=10
                #self.kick_call()
            else:
                self.cmd[id].kick_speed_x=0
                #self.kick_end_call()

    """
    def kick_client(self):
        rospy.wait_for_service("robot_0/kick")
        rospy.wait_for_service("robot_0/kick")
        self.kick_call = rospy.ServiceProxy("robot_0/kick", Kick)
        self.kick_end_call = rospy.ServiceProxy("robot_0/kick_end", Kick)
    """


    """---ドリブル判定---"""
    def dribble(self):
        None

    """---pid(pd)による移動(1台)---"""
    def pid(self, id):
        self.robot[id].goal_pos_x, self.robot[id].goal_pos_y, self.robot[id].goal_pos_theta = self.robot[id].get_future_position()
        self.Kpv = 3
        self.Kpr = 7
        self.Kdv = 1.2
        self.Kdr = 5
        current_position_x, current_position_y, current_orientation = self.robot[id].get_current_position()
        d_x = self.robot[id].goal_pos_x - current_position_x
        d_y = self.robot[id].goal_pos_y - current_position_y
        d_theta = self.robot[id].goal_pos_theta - current_orientation
        if self.robot[id].pid_init_flag:
            self.robot[id].pid_init_flag = False
            self.robot[id].pid_d_x = d_x
            self.robot[id].pid_d_y = d_y
            self.robot[id].pid_d_theta = d_theta
            self.robot[id].pid_p_prev_x = d_x
            self.robot[id].pid_p_prev_y = d_y
            self.robot[id].pid_p_prev_theta = d_theta
        else:
            self.robot[id].pid_d_x = d_x - self.robot[id].pid_p_prev_x
            self.robot[id].pid_d_y = d_y - self.robot[id].pid_p_prev_y
            self.robot[id].pid_d_theta = d_theta - self.robot[id].pid_p_prev_theta

            self.robot[id].pid_p_prev_x = d_x
            self.robot[id].pid_p_prev_y = d_y
            self.robot[id].pid_p_prev_theta = d_theta

        self.robot[id].pid_p_x = d_x
        self.robot[id].pid_p_y = d_y
        self.robot[id].pid_p_theta = d_theta

        Vx = self.Kpv * self.robot[id].pid_p_x + self.Kdv * self.robot[id].pid_d_x / (1./100)
        Vy = self.Kpv * self.robot[id].pid_p_y + self.Kdv * self.robot[id].pid_d_y / (1./100)
        Vr = self.Kpr * self.robot[id].pid_p_theta + self.Kdr * self.robot[id].pid_d_theta / (1./100)

        #self.cmd[id] = robot_commands()
        self.cmd[id].vel_surge=Vx*math.cos(current_orientation)+Vy*math.sin(current_orientation)
        self.cmd[id].vel_sway=-Vx*math.sin(current_orientation)+Vy*math.cos(current_orientation)
        self.cmd[id].omega=Vr
        #self.cmd[id].kick_speed_x=0
        #self.cmd[id].kick_speed_z=0
        #self.cmd[id].dribble_power=0
        #self.robot_cmd_publisher(id)

    """---pid(pd)による移動(全機)---"""
    def pid_all_2(self):
        for id in range(self.robot_num):
            self.pid(id)

    """---全機をストップする---"""
    def stop_all(self):
        for id in range(self.robot_num):
            self.cmd[id].vel_surge = 0.
            self.cmd[id].vel_sway = 0.
            self.cmd[id].kick_speed_x=0
            self.cmd[id].dribble_power=0

    """---ボールが50cmよりも近い場合に離れる---"""
    def leave_from_ball(self):
        ball_x, ball_y, _ = self.ball.get_current_position()
        for id in range(self.robot_num):
            robot_x, robot_y, robot_theta = self.robot[id].get_current_position()
            if (ball_x - robot_x)**2 + (ball_y - robot_y)**2 < (self.robot_r + 0.5)**2:
                length = np.sqrt((ball_x - robot_x)**2 + (ball_y - robot_y)**2)
                target_x = ball_x + 0.6 * (robot_x - ball_x) / length
                target_y = ball_y + 0.6 * (robot_y - ball_y) / length
                self.robot[id].set_future_position(target_x, target_y, robot_theta)
                self.pid(id)
            else:
                self.cmd[id].vel_surge = 0.
                self.cmd[id].vel_sway = 0.
                self.cmd[id].kick_speed_x=0
                self.cmd[id].dribble_power=0

    """---キックオフ時のPosition(20181216時点では3番しか動かない)---"""
    def begin_play(self):
        self.robot[3].set_future_position(0., 0., 0.)
        self.pid(3)
        #robot_x, robot_y, _ = self.robot[3].get_current_position()
        #ball_x, ball_y, _ = self.ball.get_current_position()
        #if (ball_x - robot_x)**2 + (ball_y - robot_y)**2 < (self.robot_r)**2:
            #self.cmd[3].kick_speed_x=1

    """---Refereeからの指示にしたがって行動指針を決定する---"""
    def referee_branch_decision(self):
        """
        branch = 0 -> STOP
        branch = 1 -> far from ball
        branch = 2 -> normal
        branch = 3 -> kickoff
        """
        if self.referee == "data: 0":
            self.referee_branch = "HALT"
        elif self.referee == "data: 1":
            self.referee_branch = "STOP"
        elif self.referee == "data: 2":
            self.referee_branch = "NORMAL_START"
        elif self.referee == "data: 3":
            self.referee_branch = "NORMAL_START"
        elif self.referee == "data: 4":
            self.referee_branch = "KICKOFF"
        elif self.referee == "data: 5":
            self.referee_branch = "DEFENCE"
        elif self.referee == "data: 6":
            self.referee_branch = "HALT"
        elif self.referee == "data: 7":
            self.referee_branch = "HALT"
        elif self.referee == "data: 8":
            self.referee_branch = "HALT"
        elif self.referee == "data: 9":
            self.referee_branch = "HALT"
        elif self.referee == "data: 10":
            self.referee_branch = "HALT"
        elif self.referee == "data: 11":
            self.referee_branch = "HALT"
        elif self.referee == "data: 12":
            self.referee_branch = "HALT"
        elif self.referee == "data: 13":
            self.referee_branch = "HALT"
        elif self.referee == "data: 14":
            self.referee_branch = "STOP"
        elif self.referee == "data: 15":
            self.referee_branch = "STOP"

        if self.color == "Blue":
            if self.referee == "data: 4":
                self.referee_branch = "DEFENCE"
            elif self.referee == "data: 5":
                self.referee_branch = "KICKOFF"

    """---cmdをバイト情報にして送信する(荒木くんがここからさらにアップデートしてくれてる)---"""
    def cmd_to_byte(self, id):
        command = format(ord("A"),"08b")
        command += format(ord("B"),"08b")
        command += format(int(abs(self.cmd[id].vel_surge * 1000)),"032b")
        command += format(int(abs(self.cmd[id].vel_sway * 1000)),"032b")
        command += format(int(abs(self.cmd[id].omega * 1000)),"032b")
        command += format(int(abs(self.cmd[id].kick_speed_x * 10)),"04b")
        command += format(int(abs(self.cmd[id].dribble_power * 10)),"04b")
        command_head1 = command[:8]
        command_head2 = command[8:16]
        command_vx = command[16:48]
        command_vy = command[48:80]
        command_omega = command[80:112]
        command_kick_speed = command[112:116]
        command_dribble_power = command[116:120]
        print(chr(int(command_head1,2)))
        print(chr(int(command_head2,2)))
        print(int(command_vx,2))
        print(int(command_vy,2))
        print(int(command_omega,2))
        print(int(command_kick_speed,2))
        print(int(command_dribble_power,2))

        self.robot[id].serial.write(command)



if __name__ == "__main__":
    a = WorldModel()
    a.odom_listener()
    a.referee_listener()
    #a.kick_client()

    print("START")

    a.set_first_positions_4robots()
    loop_rate = rospy.Rate(WORLD_LOOP_RATE)


    """
    counter = 0
    try:
        while not rospy.is_shutdown():
            print(a.referee_branch)
            a.referee_branch_decision()
            if a.referee_branch == "HALT":
                a.stop_all()
            elif a.referee_branch == "STOP":
                a.leave_from_ball()
            elif a.referee_branch == "NORMAL_START":
                a.who_has_a_ball()
                a.kick()
                a.update_strategy()
                a.pid_all_2()
            elif a.referee_branch == "KICKOFF":
                a.set_first_position_4robots_attack()
                a.pid_all_2()
            elif a.referee_branch == "DEFENCE":
                a.set_first_positions_4robots()
                a.pid_all_2()
            a.robot_collision_detect_2()
            #if counter > 100:
            a.robot_cmd_publisher_4robots()
            counter += 1
            loop_rate.sleep()
    except rospy.ROSInterruptException:
        print("check")
        a.stop_all()
        a.robot_cmd_publisher_4robots()
        sys.exit()
    """
    
    while not rospy.is_shutdown():
        #a.robot_status_publisher()
        time.sleep(0.1)
