#!/usr/bin/env  python
# coding:utf-8
import math
import rospy
from consai_msgs.msg import Pose
from consai_msgs.msg import robot_commands
#from aisaac.srv import Kick
from aisaac.msg import Status
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import entity

ROBOT_LOOP_RATE = 60.

class RobotParameters:
    def __init__(self, robot_num,friend_num, enemy_num):
        self.robot_r = 0.09 #定数はconfigからとってくるように書きたい

        self.robot_num = int(robot_num)
        self.friend_num = friend_num
        self.enemy_num = enemy_num

        self.friend = [entity.Robot() for i in range(int(self.friend_num))]
        self.enemy = [entity.Robot() for i in range(int(self.enemy_num))]

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.pid_goal_pos_x = 0
        self.pid_goal_pos_y = 0
        self.pid_goal_theta = 0

        self.pass_target_pos_x = 0
        self.pass_target_pos_y = 0

    def friend_odom_callback(self, msg, id):
        friend_x = msg.pose.pose.position.x
        friend_y = msg.pose.pose.position.y
        friend_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.friend[id].set_current_position(x = friend_x, y = friend_y, theta = friend_t[2])
        if self.robot_num == id:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            current_ori = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
            self.current_theta = current_ori[2]

    def enemy_odom_callback(self, msg, id):
        enemy_x = msg.pose.pose.position.x
        enemy_y = msg.pose.pose.position.y
        enemy_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.enemy[id].set_current_position(x = enemy_x, y = enemy_y, theta = enemy_t[2])

        """

    def goal_pose_callback(self, msg):
        if self.goal_pos_x != msg.x or self.goal_pos_y != msg.y or self.goal_pos_theta != msg.theta:
           self.goal_pos_x = msg.x
           self.goal_pos_y = msg.y
           self.goal_pos_theta = msg.theta
           self.pid_circle_center_x = (self.goal_pos_x + self.current_x) / 2
           self.pid_circle_center_y = (self.goal_pos_y + self.current_y) / 2
           self.goal_pos_init_flag = True

    def target_pose_callback(self, msg):
        self.target_pos_x = msg.x
        self.target_pos_y = msg.y

        """


class RobotPid:
    def __init__(self, robot_params, ball_params, cmd, command_pub):
        self.robot_params = robot_params
        self.ball_params = ball_params
        self.cmd = cmd
        self.command_pub = command_pub

        self.goal_pos_init_flag = True

        self.pid_circle_center_x = 0
        self.pid_circle_center_y = 0

        self.recursion_max = 10
        self.recursion_count = 0

    """---点と直線の距離の計算---"""
    def distance_of_a_point_and_a_straight_line(self, x_0, y_0, a, b, c):
        d = 0
        if a != 0 or b != 0:
            d = abs(a * x_0 + b * y_0 + c) / math.sqrt(a**2 + b**2)
        return d

    """---2点をつなぐ直線ax+by+cのa,b,cを返す---"""
    def line_parameters(self, x_1, y_1, x_2, y_2):
        a = y_1 - y_2
        b = x_2 - x_1
        c = x_1 * y_2 - x_2 * y_1
        return a, b, c

    def collision_Detection(self, goal_pos_x, goal_pos_y):
        a, b, c = self.line_parameters(self.robot_params.current_x, self.robot_params.current_y, goal_pos_x, goal_pos_y)
        if a != 0 and b != 0:
            for i in range(self.robot_params.friend_num):
                if i != self.robot_params.robot_num:
                    distance = self.distance_of_a_point_and_a_straight_line(self.robot_params.friend[i].current_position_x, self.robot_params.friend[i].current_position_y, a, b, c)
                    if distance < self.robot_params.robot_r * 3:
                        x = (-self.robot_params.friend[i].current_position_y * b + (b**2 / a) * self.robot_params.friend[i].current_position_x - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.robot_params.current_x < x < goal_pos_x or self.robot_params.current_x > x > goal_pos_x) and (self.robot_params.current_y < y < goal_pos_y or self.robot_params.current_y > y > goal_pos_y):
                            return True, x, y, self.robot_params.friend[i].current_position_x , self.robot_params.friend[i].current_position_y, distance
            for j in range(self.robot_params.enemy_num):
                distance = self.distance_of_a_point_and_a_straight_line(self.robot_params.enemy[j].current_position_x, self.robot_params.enemy[j].current_position_y, a, b, c)
                if distance < self.robot_params.robot_r * 3:
                        x = (-self.robot_params.enemy[j].current_position_y * b + (b**2 / a) * self.robot_params.enemy[j].current_position_x - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.robot_params.current_x < x < goal_pos_x or self.robot_params.current_x > x > goal_pos_x) and (self.robot_params.current_y < y < goal_pos_y or self.robot_params.current_y > y > goal_pos_y):
                            return True, x, y, self.robot_params.enemy[j].current_position_x , self.robot_params.enemy[j].current_position_y, distance

            distance = self.distance_of_a_point_and_a_straight_line(self.ball_params.ball_pos_x, self.ball_params.ball_pos_y, a, b, c)
            if distance < self.robot_params.robot_r * 2:
                        x = (-self.ball_params.ball_pos_y * b + (b**2 / a) * self.ball_params.ball_pos_x - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.robot_params.current_x < x < goal_pos_x or self.robot_params.current_x > x > goal_pos_x) and (self.robot_params.current_y < y < goal_pos_y or self.robot_params.current_y > y > goal_pos_y):
                            return True, x, y, self.ball_params.ball_pos_x , self.ball_params.ball_pos_y, distance
        return False, 0, 0, 0, 0, 0

    def get_sub_goal(self, x, y, obstacle_x, obstacle_y, distance):
        if distance != 0:
            sub_goal_x = (-5 * self.robot_params.robot_r * obstacle_x + (distance + 5 * self.robot_params.robot_r) * x) / distance
            sub_goal_y = (-5 * self.robot_params.robot_r * obstacle_y + (distance + 5 * self.robot_params.robot_r) * y) / distance
            return sub_goal_x, sub_goal_y
        else :
            return 0, 0

    def pass_plan(self, goal_pos_x, goal_pos_y):
        self.recursion_count += 1
        collision = self.collision_Detection(goal_pos_x, goal_pos_y)
        if collision[0] and self.recursion_count < self.recursion_max:
            goal_pos_x, goal_pos_y = self.get_sub_goal(collision[1], collision[2], collision[3], collision[4], collision[5])
            self.pass_plan(goal_pos_x, goal_pos_y)
        return goal_pos_x, goal_pos_y

    def pid_linear(self, goal_pos_x, goal_pos_y, goal_pos_theta):
        self.Kpv = 2.2
        self.Kpr = 4
        self.Kdv = 5
        self.Kdr = 3

        """
        self.recursion_count = 0
        next_pos_x, next_pos_y = self.pass_plan(goal_pos_x, goal_pos_y)
        if goal_pos_x != next_pos_x or goal_pos_y != next_pos_y:
            self.goal_pos_init_flag = True
            goal_pos_x = next_pos_x
            goal_pos_y = next_pos_y
        """


        if self.goal_pos_init_flag == True:
            self.recursion_count = 0
            self.next_pos_x, self.next_pos_y = self.pass_plan(goal_pos_x, goal_pos_y)
            """
            if goal_pos_x != next_pos_x or goal_pos_y != next_pos_y:
                goal_pos_x = next_pos_x
                goal_pos_y = next_pos_y
            """


        d_x = self.next_pos_x - self.robot_params.current_x
        d_y = self.next_pos_y - self.robot_params.current_y
        d_theta = goal_pos_theta - self.robot_params.current_theta
        if self.goal_pos_init_flag:
            self.goal_pos_init_flag = False
            self.pid_d_x = 0
            self.pid_d_y = 0
            self.pid_d_theta = 0
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

    def pid_circle(self, center_x, center_y, x, y, theta):
        self.Kpv = 2
        self.Kpr = 7
        self.Kdr = 4
        d_x = center_x - self.robot_params.current_x
        d_y = center_y - self.robot_params.current_y
        d_theta = theta - self.robot_params.current_theta
        if self.goal_pos_init_flag:
            self.goal_pos_init_flag = False
            self.pid_d_theta = d_theta
            self.pid_p_prev_theta = d_theta
        else:
            self.pid_d_theta = d_theta - self.pid_p_prev_theta
            self.pid_p_prev_theta = d_theta

        self.pid_p_x = d_x
        self.pid_p_y = d_y
        self.pid_p_theta = d_theta

        Vx = 0
        Vy = 0
        Vr = 0

        Vx_tangent = 0
        Vy_tangent = 0

        if math.sqrt(d_x * d_x + d_y * d_y) != 0:
            Vx = self.Kpv * self.pid_p_x
            Vy = self.Kpv * self.pid_p_y
            Vr = self.Kpr * self.pid_p_theta + self.Kdr * self.pid_d_theta / (1./ROBOT_LOOP_RATE)
            Vx_tangent = (-d_y / math.sqrt(d_x * d_x + d_y * d_y)) * 2
            Vy_tangent = (d_x / math.sqrt(d_x * d_x + d_y * d_y)) * 2

            self.cmd.vel_surge=(Vx + Vx_tangent)*math.cos(self.robot_params.current_theta)+(Vy + Vy_tangent)*math.sin(self.robot_params.current_theta)
            self.cmd.vel_sway=-(Vx + Vx_tangent)*math.sin(self.robot_params.current_theta)+(Vy + Vy_tangent)*math.cos(self.robot_params.current_theta)
            self.cmd.omega=Vr
            self.command_pub.publish(self.cmd)

    def replan_timerCallback(self, event):
        self.goal_pos_init_flag = True

class Ball:
    def __init__(self):
        self.ball_pos_x = 0
        self.ball_pos_y = 0
        self.ball_pos_theta = 0

    def odom_callback(self, msg):
        self.ball_pos_x = msg.pose.pose.position.x
        self.ball_pos_y = msg.pose.pose.position.y
        self.ball_pos_theta = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

class RobotStatus:
    def __init__(self, pid, robot_params):
        self.robot_status = "pass"
        
        self.pid = pid

        self.robot_params = robot_params
        self.robot_params.pid_goal_pos_x = 0
        self.robot_params.pid_goal_pos_y = 0
        self.robot_params.pid_goal_theta = 0

    def status_callback(self, msg):
        if msg.status == "move_linear" and (self.robot_params.pid_goal_pos_x != msg.pid_goal_pos_x or self.robot_params.pid_goal_pos_y != msg.pid_goal_pos_y or self.robot_params.pid_goal_theta != msg.pid_goal_theta):
            self.pid.goal_pos_init_flag = True
        self.robot_status = msg.status
        self.robot_params.pid_goal_pos_x = msg.pid_goal_pos_x
        self.robot_params.pid_goal_pos_y = msg.pid_goal_pos_y
        self.robot_params.pid_goal_theta = msg.pid_goal_theta
        self.pid.pid_circle_center_x = msg.pid_circle_center_x
        self.pid.pid_circle_center_y = msg.pid_circle_center_y
        self.robot_params.pass_target_pos_x = msg.pass_target_pos_x
        self.robot_params.pass_target_pos_y = msg.pass_target_pos_y

class RobotKick:
    def __init__(self, ball_params, robot_params, pid, cmd, status, command_pub):
        self.kick_power_x = 10
        self.kick_power_z = 0

        self.ball_params = ball_params
        self.robot_params = robot_params
        self.pid = pid
        self.status = status
        self.cmd = cmd
        self.command_pub = command_pub
        self.dispersion = [10] * 100

    def kick_x(self):
        area = 0.3
        if math.sqrt((self.ball_params.ball_pos_x - self.robot_params.current_x)**2 + (self.ball_params.ball_pos_y - self.robot_params.current_y)**2) > self.robot_params.robot_r + area:
            self.cmd.vel_surge = 0
            self.cmd.vel_sway = 0
            self.cmd.omega = 0
            self.cmd.kick_speed_x = 0
            self.command_pub.publish(self.cmd)
            self.status.robot_status = "None"
            return

        self.cmd.kick_speed_x = self.kick_power_x
        self.pid.pid_linear(self.ball_params.ball_pos_x, self.ball_params.ball_pos_y, math.atan2( (self.ball_params.ball_pos_y - self.robot_params.current_y) , (self.ball_params.ball_pos_x - self.robot_params.current_x) ))

    def pass_ball(self, target_x, target_y):
        distance = math.sqrt((target_x - self.ball_params.ball_pos_x)**2 + (target_y - self.ball_params.ball_pos_y)**2)
        if distance != 0:
            pose_x = (- 0.15 * target_x + (0.15 + distance) * self.ball_params.ball_pos_x) / distance
            pose_y = (- 0.15 * target_y + (0.15 + distance) * self.ball_params.ball_pos_y) / distance
            pose_theta = math.atan2( (target_y - self.robot_params.current_y) , (target_x - self.robot_params.current_x) )

            self.dispersion.append((self.ball_params.ball_pos_x - self.robot_params.current_x)**2 + (self.ball_params.ball_pos_y - self.robot_params.current_y)**2)
            del self.dispersion[0]

            if sum(self.dispersion) < 3:
                self.kick_power_x = math.sqrt(distance) * 4.0
                self.status.robot_status = "kick"
                return

            self.pid.pid_linear(pose_x, pose_y, pose_theta)

    def kick_z(self):
        pass

class RobotStrategy:
    def __init__(self):
        pass

    def attack_startegy(self):
        pass
