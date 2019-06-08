# !/usr/bin/env  python
# coding:utf-8
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

class WorldState:
    def __init__(self, objects):
        self.robot = objects.robot
        self.robot_total = objects.robot_total

        """---devisionの選択---"""
        self.devision = "A"
        #self.devision = "B"

        """---フィールドとロボットのパラメータ---"""
        self.robot_r = objects.robot[0].size_r
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

        """positionの割り振り(動的にPositionが切り替わるのは多分大事、現状使っていない)"""
        if self.robot_total == 8:
            self.robot[0].position = "GK"
            self.robot[1].position = "LCB"
            self.robot[2].position = "RCB"
            self.robot[3].position = "LSB"
            self.robot[4].position = "RSB"
            self.robot[5].position = "LMF"
            self.robot[6].position = "RMF"
            self.robot[7].position = "CF"
        elif self.robot_total == 4:
            self.robot[0].position = "GK"
            self.robot[1].position = "LCB"
            self.robot[2].position = "CCB"
            self.robot[3].position = "RCB"


class Referee:
    def __init__(self, world_state):
        self.world_state = world_state
        """---refereeからの指示をもとにどのループを回すかの指標---"""
        self.referee_branch = None

        """refereeから受信する情報"""
        self.command = None
        self.stage = None
        self.teaminfo = None

    """---Refereeから現在のcommandをもらう---"""
    def command_callback(self, msg):
        self.command = str(msg)

    """---Refereeから現在のstageをもらう---"""
    def stage_callback(self, msg):
        self.stage = str(msg)
        # ハーフタイムかどうかなどの司令を受け取る。

    """---Refereeから現在のteaminfoをもらう---"""
    def teaminfo_callback(self, msg):
        self.teaminfo = str(msg)
        # 各機体にイエローカード、レッドカードなどがあることをしらせる。

    """---Refereeからの指示にしたがって行動指針を決定する---"""
    def referee_branch_decision(self):
        """
        branch = 0 -> STOP
        branch = 1 -> far from ball
        branch = 2 -> normal
        branch = 3 -> kickoff
        """
        #   print(self.command)
        if self.command == "data: 0":
            # Robots must keep 50 cm from the ball.
            self.referee_branch = "HALT"
        elif self.command == "data: 1":
            # A prepared kickoff or penalty may now be taken.
            self.referee_branch = "STOP"
        elif self.command == "data: 2":
            # The ball is dropped and free for either team.
            self.referee_branch = "NORMAL_START"

        elif self.command == "data: 3":
            # FORCE_START = 3
		    #The yellow team may move into kickoff position.
            self.referee_branch = "NORMAL_START"
        elif self.command == "data: 4":
            # PREPARE_KICKOFF_YELLOW = 4;
		    #The blue team may move into kickoff position.
            self.referee_branch = "KICKOFF"
        elif self.command == "data: 5":
            # PREPARE_KICKOFF_BLUE = 5;
		    # The yellow team may move into penalty position.
            self.referee_branch = "DEFENCE"
        elif self.command == "data: 6":
            # PREPARE_PENALTY_YELLOW = 6;
		    # The blue team may move into penalty position.
            self.referee_branch = "HALT"
        elif self.command == "data: 7":
            # PREPARE_PENALTY_BLUE = 7;
		    # The yellow team may take a direct free kick.
            self.referee_branch = "HALT"
        elif self.command == "data: 8":
            # DIRECT_FREE_YELLOW = 8;
		    # The blue team may take a direct free kick.
            self.referee_branch = "HALT"
        elif self.command == "data: 9":
            #DIRECT_FREE_BLUE = 9;
		    # The yellow team may take an indirect free kick.
            self.referee_branch = "HALT"
        elif self.command == "data: 10":
            #INDIRECT_FREE_YELLOW = 10;
		    # The blue team may take an indirect free kick.
            self.referee_branch = "HALT"
        elif self.command == "data: 11":
            # INDIRECT_FREE_BLUE = 11;
		    # The yellow team is currently in a timeout.
            self.referee_branch = "HALT"
        elif self.command == "data: 12":
            # TIMEOUT_YELLOW = 12;
		    # The blue team is currently in a timeout.
            self.referee_branch = "HALT"
        elif self.command == "data: 13":
            # TIMEOUT_BLUE = 13;
		    # The yellow team just scored a goal.
		    # For information only.
		    # For rules compliance, teams must treat as STOP.
            self.referee_branch = "HALT"
        elif self.command == "data: 14":
            # GOAL_YELLOW = 14;
		    # The blue team just scored a goal.
            self.referee_branch = "STOP"
        elif self.command == "data: 15":
            # GOAL_BLUE = 15;
            # Equivalent to STOP, but the yellow team must pick up the ball and
		    # drop it in the Designated Position.
            self.referee_branch = "STOP"
        elif self.command == "data: 16":
            #BALL_PLACEMENT_YELLOW = 16;
	        #Equivalent to STOP, but the blue team must pick up the ball and drop
		    # it in the Designated Position.
            self.referee_branch = "STOP"
        elif self.command == "data: 17":
            #BALL_PLACEMENT_BLUE = 17;
            self.referee_branch = "STOP"

        if self.world_state.color == "Blue":
            if self.command == "data: 4":
                self.referee_branch = "DEFENCE"
            elif self.command == "data: 5":
                self.referee_branch = "KICKOFF"

class DecisionMaker:
    def __init__(self, world_state, objects, referee, status):
        self.world_state = world_state
        self.robot_total = objects.robot_total
        self.enemy_total = objects.enemy_total
        self.robot = objects.robot
        self.enemy = objects.enemy
        self.ball = objects.ball
        self.ball_dynamics_window = objects.ball_dynamics_window
        self.ball_dynamics = objects.ball_dynamics
        self.referee_branch = referee.referee_branch
        self.status = status
        self.robot_r = objects.robot[0].size_r

        self.goal_keeper_x = self.world_state.field_x_min + 0.05

        """strategyの選択(いまはつかっていない)"""
        self.strategy = None

        """---敵と味方どっちがボールをもっているか(現状使ってない)---"""
        self.which_has_a_ball = None

        """---攻撃かdefenceか(これをうまく使っていきたい)---"""
        self.attack_or_defence = None

    """future positionをstatusにつめる"""
    def change_goal_status(self):
        for id in range(self.robot_total):
            self.status[id].pid_goal_pos_x, self.status[id].pid_goal_pos_y, self.status[id].pid_goal_theta = self.robot[id].get_future_position()


    """誰がボールを持っているかの判定"""
    def who_has_a_ball(self):
        x_ball, y_ball, _ = self.ball.get_current_position()
        flag = 0
        area = 0.015
        for i in range(self.robot_total):
            x_robot, y_robot, _ = self.robot[i].get_current_position()
            if (x_ball - x_robot)**2 + (y_ball - y_robot)**2 < (self.robot_r + area)**2:
                self.robot[i].has_a_ball = True
                #print("check:",i)
                flag = flag + 1
            else:
                self.robot[i].has_a_ball = False
        for i in range(self.robot_total):
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

    """---GKは常に直線フィッティングの先へと移動する---"""
    def goal_keeper_strategy(self):
        a, b = self.ball_liner_fitting()
        y = a * self.goal_keeper_x + b
        if y > self.world_state.goal_y_min and y < self.world_state.goal_y_max:
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

    """---20181216時の戦略---"""
    def update_strategy(self):
        robot_x, robot_y, _ = self.robot[3].get_current_position()
        ball_x, ball_y, _ = self.ball.get_current_position()
        close_range = 0.3
        if robot_x**2 + robot_y**2 < (self.robot_r + close_range) **2:
            theta = np.arctan( (0. - robot_y) / (self.world_state.field_x_max - robot_x) )
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
                        a, b, c = functions.line_parameters(x_1=-6., y_1=0., x_2=enemy_x, y_2=enemy_y)
                        target_x, target_y = functions.calculate_contact(a, b, c, robot_x, robot_y)
                        theta = np.arctan(- a / b)
                        self.robot[1].set_future_position(target_x, target_y, theta)
                        self.robot[2].set_future_position(-4., -1.5, 0.)
                elif ball_flag == "area2" or ball_flag == "area4":
                    if data[1] == "area2" or data[1] == "area4":
                        #print("ok2")
                        enemy_x, enemy_y, _ = self.enemy[data[0]].get_current_position()
                        robot_x, robot_y ,_ = self.robot[2].get_current_position()
                        a, b, c = functions.line_parameters(x_1=-6., y_1=0., x_2=enemy_x, y_2=enemy_y)
                        target_x, target_y = functions.calculate_contact(a, b, c, robot_x, robot_y)
                        theta = np.arctan(- a / b)
                        self.robot[2].set_future_position(target_x, target_y, theta)
                        self.robot[1].set_future_position(-4., 1.5, 0.)
                else:
                    self.robot[1].set_future_position(-4., 1.5, 0.)
                    self.robot[2].set_future_position(-4., -1.5, 0.)
            self.goal_keeper_strategy()


    """---敵のPositionを確認する---"""
    def check_enemy_position(self):
        self.enemy_info = []
        for id in range(self.enemy_total):
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

    """---ボールの軌道に直線をフィッティング y=ax+bのaとbを返す---"""
    def ball_liner_fitting(self):
        while len(self.ball_dynamics) != self.ball_dynamics_window:
            if len(self.ball_dynamics) < self.ball_dynamics_window:
                self.ball_dynamics.append([0., 0.])
            else:
                _ = self.ball_dynamics.pop(0)


        array_x = np.array([x[0] for x in self.ball_dynamics])
        array_y = np.array([x[1] for x in self.ball_dynamics])
        n = self.ball_dynamics_window
        xy_sum = np.dot(array_x, array_y)
        x_sum = np.sum(array_x)
        y_sum = np.sum(array_y)
        x_square_sum = np.dot(array_x, array_x)
        a = (n * xy_sum - x_sum * y_sum) / (n * x_square_sum - (x_sum ** 2) + 0.00001)
        b = (x_square_sum * y_sum - xy_sum * x_sum) / (n * x_square_sum - x_sum ** 2 + 0.00001)
        return a,b

    """---衝突を判定して避ける動きをする---"""
    def robot_collision_back(self):
        safety_rate = 1.1
        robot = self.robot + self.enemy
        for i in range(self.robot_total):
            position_1_x, position_1_y, _ = robot[i].get_current_position()
            for j in range(self.robot_total*2-i-1):
                position_2_x, position_2_y, _ = robot[j+i+1].get_current_position()
                if (position_1_x - position_2_x)**2 + (position_1_y - position_2_y)**2 < (self.robot_r * 2 * 1.1)**2:
                    self.status[i].status = 'collision'
                    self.status[i].pid_goal_pos_x = 2 * position_1_x - position_2_x
                    self.status[i].pid_goal_pos_y = 2 * position_1_y - position_2_y

    def stop_all(self):
        for id in range(self.robot_total):
            x, y, theta = self.robot[id].get_current_position()
            self.robot[id].set_future_position(x, y, theta)
            self.status[id].status = 'stop'

    def leave_from_ball(self):
        ball_x, ball_y, _ = self.ball.get_current_position()
        for id in range(self.robot_total):
            robot_x, robot_y, robot_theta = self.robot[id].get_current_position()
            if (ball_x - robot_x)**2 + (ball_y - robot_y)**2 < (self.robot_r + 0.5)**2:
                length = math.sqrt((ball_x - robot_x)**2 + (ball_y - robot_y)**2) + 0.001
                target_x = ball_x + 0.6 * ( (robot_x - ball_x) / length )
                target_y = ball_y + 0.6 * ( (robot_y - ball_y) / length )
                self.robot[id].set_future_position(target_x, target_y, robot_theta)
                self.status[id].status = 'move_linear'
            else:
                self.stop_all()

    """---2点をつなぐ直線ax+by+cのa,b,cを返す---"""
    def line_parameters(self, x_1, y_1, x_2, y_2):
        a = y_1 - y_2
        b = x_2 - x_1
        c = x_1 * y_2 - x_2 * y_1
        return a, b, c

    """---点と直線の距離の計算---"""
    def distance_of_a_point_and_a_straight_line(self, x_0, y_0, a, b, c):
        d = abs(a * x_0 + b * y_0 + c) / np.sqrt(a**2 + b**2)
        return d

    def count_collision(self, robot_id, current_pos_x, current_pos_y, goal_pos_x, goal_pos_y):
        counter = 0
        a, b, c = self.line_parameters(current_pos_x, current_pos_y, goal_pos_x, goal_pos_y)
        if a != 0 and b != 0:
            for i in range(self.robot_total):
                if i != robot_id:
                    friend_pos_x, friend_pos_y, _ = self.robot[i].get_current_position()
                    distance = self.distance_of_a_point_and_a_straight_line(friend_pos_x, friend_pos_y, a, b, c)
                    if distance < self.robot_r * 3:
                        x = (-friend_pos_y * b + (b**2 / a) * friend_pos_x - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (current_pos_x < x < goal_pos_x or current_pos_x > x > goal_pos_x) and (current_pos_y < y < goal_pos_y or current_pos_y > y > goal_pos_y):
                            counter += 1
            for j in range(self.enemy_total):
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
        distance = np.sqrt( (goal_x - current_x)**2. + (goal_y - current_y)**2. )
        #velocity_difference = np.sqrt( (((goal_x - current_x) * 3.7 / distance) - current_vx)**2 + (((goal_y - current_y) * 3.7 / distance) - current_vy)**2 )
        collision = self.count_collision(robot_id, current_x, current_y, goal_x, goal_y)
        velocity_difference = 0.
                
        move_cost = distance * 1.# + velocity_difference * 0.1 + collision * 3.

        return move_cost

    def goal_assignment(self, assignment_x, assignment_y, assignment_theta):
        best_cost = 100.
        best_id = 0
        used_id = []
        for  priority in range(len(assignment_x)):
            for robot_id in range(self.robot_total):
                current_cost = self.calculate_move_cost(robot_id, assignment_x[priority], assignment_y[priority])
                if (best_cost > current_cost) and (str(robot_id) not in used_id):
                    best_cost = current_cost
                    best_id = robot_id
            used_id.append(str(best_id))
            best_cost = 100.
            self.status[best_id].pid_goal_pos_x = assignment_x[priority]
            self.status[best_id].pid_goal_pos_y = assignment_y[priority]
            self.status[best_id].pid_goal_theta = assignment_theta[priority]
            self.status[best_id].status = "move_linear"
            #print best_id
            #print self.status[best_id].pid_goal_pos_x
            #print self.status[best_id].pid_goal_pos_y
            #print self.status[best_id].pid_goal_theta



class Mesh:
    def __init__(self, world_state, objects):
        self.world_state = world_state
        self.robot_total = objects.robot_total
        self.enemy_total = objects.enemy_total
        self.robot = objects.robot
        self.enemy = objects.enemy

    """フィールドをメッシュ化する(クラスタリング用に書かれた)"""
    def create_mesh(self):
        self.mesh_size = 50
        self.mesh_x_num = int(self.world_state.field_x_size / (2*self.mesh_size)) + 1
        self.mesh_y_num = int(self.world_state.field_y_size / self.mesh_size) + 1
        self.mesh_x = [ i * self.mesh_size for i in range(self.mesh_x_num)]
        self.mesh_y = [- self.world_state.field_y_size / 2 + i * self.mesh_size for i in range(self.mesh_y_num)]
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
        for i in range(self.robot_total-1):
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

        plt.scatter(centers[:, 0], centers[:, 1], s=100,facecolors='none', edgecolors='black')
        plt.xlim(-self.world_state.field_x_size/2, self.world_state.field_x_size/2)
        plt.ylim(-self.world_state.field_y_size/2, self.world_state.field_y_size/2)
        plt.show()
        #"""
