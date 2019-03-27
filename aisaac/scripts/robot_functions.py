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

ROBOT_LOOP_RATE = 60.

class RobotParameters:
    def __init__(self, robot_num):
        self.robot_r = 0.09 #定数はconfigからとってくるように書きたい

        self.robot_num = robot_num

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.pid_goal_pos_x = 0
        self.pid_goal_pos_y = 0
        self.pid_goal_theta = 0

        self.pass_target_pos_x = 0
        self.pass_target_pos_y = 0

    def odom_callback(self, msg):
    	self.current_x = msg.pose.pose.position.x
    	self.current_y = msg.pose.pose.position.y
    	current_ori = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
    	self.current_theta = current_ori[2]

        """

    def goal_pose_callback(self, msg):
        if self.goal_pos_x != msg.x or self.goal_pos_y != msg.y or self.goal_pos_theta != msg.theta:
    	   self.goal_pos_x = msg.x
    	   self.goal_pos_y = msg.y
    	   self.goal_pos_theta = msg.theta
           self.pid_circle_center_x = (self.goal_pos_x + self.current_x) / 2
           self.pid_circle_center_y = (self.goal_pos_y + self.current_y) / 2
    	   self.pid_init_flag = True

    def target_pose_callback(self, msg):
        self.target_pos_x = msg.x
        self.target_pos_y = msg.y

        """


class RobotMove:
    def __init__(self, robot_params, cmd, command_pub):
        self.robot_params = robot_params
        self.cmd = cmd
        self.command_pub = command_pub

        self.pid_init_flag = True

        self.pid_circle_center_x = 0
        self.pid_circle_center_y = 0


    def pid_linear(self, x, y, theta):
        self.Kpv = 2.8
        self.Kpr = 4
        self.Kdv = 5
        self.Kdr = 3
        d_x = x - self.robot_params.current_x
        d_y = y - self.robot_params.current_y
        d_theta = theta - self.robot_params.current_theta
        if self.pid_init_flag:
            self.pid_init_flag = False
            self.pid_d_x = d_x
            self.pid_d_y = d_y
            self.pid_d_theta = d_theta
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

        self.cmd.vel_surge=Vx*math.cos(self.robot_params.current_theta)+Vy*math.sin(self.robot_params.current_theta)
        self.cmd.vel_sway=-Vx*math.sin(self.robot_params.current_theta)+Vy*math.cos(self.robot_params.current_theta)
        self.cmd.omega=Vr
        self.command_pub.publish(self.cmd)

    def pid_circle(self, center_x, center_y, x, y, theta):
        self.Kpv = 2
        self.Kpr = 7
        self.Kdr = 4
        d_x = center_x - self.robot_params.current_x
        d_y = center_y - self.robot_params.current_y
        d_theta = theta - self.robot_params.current_theta
        if self.pid_init_flag:
            self.pid_init_flag = False
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

    def stop(self):
        self.cmd.vel_surge = 0.
        self.cmd.vel_sway = 0.
        self.cmd.kick_speed_x=0
        self.cmd.dribble_power=0
        self.command_pub.publish(self.cmd)


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
    def __init__(self, move, robot_params):
        self.robot_status = "None"

        self.move = move

        self.robot_params = robot_params
        self.robot_params.pid_goal_pos_x = 0
        self.robot_params.pid_goal_pos_y = 0
        self.robot_params.pid_goal_theta = 0

    def status_callback(self, msg):
        if msg.status == "move_linear" and (self.robot_params.pid_goal_pos_x != msg.pid_goal_pos_x or self.robot_params.pid_goal_pos_y != msg.pid_goal_pos_y or self.robot_params.pid_goal_theta != msg.pid_goal_theta):
            self.move.pid_init_flag = True
        self.robot_status = msg.status
        self.robot_params.pid_goal_pos_x = msg.pid_goal_pos_x
        self.robot_params.pid_goal_pos_y = msg.pid_goal_pos_y
        self.robot_params.pid_goal_theta = msg.pid_goal_theta
        self.move.pid_circle_center_x = msg.pid_circle_center_x
        self.move.pid_circle_center_y = msg.pid_circle_center_y
        self.robot_params.pass_target_pos_x = msg.pass_target_pos_x
        self.robot_params.pass_target_pos_y = msg.pass_target_pos_y

class RobotKick:
    def __init__(self, ball_params, robot_params, move, cmd, status, command_pub):
        self.kick_power_x = 10
        self.kick_power_z = 0

        self.ball_params = ball_params
        self.robot_params = robot_params
        self.move = move
        self.status = status
        self.cmd = cmd
        self.command_pub = command_pub
        self.dispersion = [10] * 100
        self.access_threshold = 3
        self.const = 4.0

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
        self.move.pid_linear(self.ball_params.ball_pos_x, self.ball_params.ball_pos_y, math.atan2( (self.ball_params.ball_pos_y - self.robot_params.current_y) , (self.ball_params.ball_pos_x - self.robot_params.current_x) ))

    def pass_ball(self, target_x, target_y):
        distance = math.sqrt((target_x - self.ball_params.ball_pos_x)**2 + (target_y - self.ball_params.ball_pos_y)**2)
        if distance != 0:
            pose_x = (- 0.15 * target_x + (0.15 + distance) * self.ball_params.ball_pos_x) / distance
            pose_y = (- 0.15 * target_y + (0.15 + distance) * self.ball_params.ball_pos_y) / distance
            pose_theta = math.atan2( (target_y - self.robot_params.current_y) , (target_x - self.robot_params.current_x) )
            #area = 0.16

            self.dispersion.append((self.ball_params.ball_pos_x - self.robot_params.current_x)**2 + (self.ball_params.ball_pos_y - self.robot_params.current_y)**2)
            del self.dispersion[0]



            if sum(self.dispersion) < self.access_threshold:
                self.kick_power_x = math.sqrt(distance) * self.const
                self.status.robot_status = "kick"
                return

            self.move.pid_linear(pose_x, pose_y, pose_theta)

    def kick_z(self):
        pass


class RobotStrategy:
    def __init__(self):
        pass

    def attack_startegy(self):
        pass
