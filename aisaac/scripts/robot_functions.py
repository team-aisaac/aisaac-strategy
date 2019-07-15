#!/usr/bin/env  python
# coding:utf-8
import itertools
import math
import rospy
import numpy as np
from consai_msgs.msg import Pose
from consai_msgs.msg import robot_commands
#from aisaac.srv import Kick
from aisaac.msg import Status
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import entity
import matplotlib.pyplot as plt
from matplotlib import animation
import functions

ROBOT_LOOP_RATE = 60.


class RobotPid(object):
    def __init__(self, robot_id, objects, cmd, command_pub):
        self.robot_id = int(robot_id)
        self.ctrld_robot = objects.robot[int(robot_id)]
        self.friend = objects.robot
        self.enemy = objects.enemy
        self.ball_params = objects.ball
        self.cmd = cmd
        self.command_pub = command_pub

        self.goal_pos_init_flag = True

        self.pid_circle_center_x = 0
        self.pid_circle_center_y = 0

        self.recursion_max = 10
        self.recursion_count = 0

        self.Kpv = 3.0
        self.Kpr = 6.0
        self.Kdv = 3.0
        self.Kdr = 4.0

    def collision_detection(self, goal_pos_x, goal_pos_y):
        a, b, c = functions.line_parameters(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], goal_pos_x, goal_pos_y)
        if a != 0 and b != 0:
            for i in range(len(self.friend)):
                if i != self.robot_id:
                    distance = functions.distance_of_a_point_and_a_straight_line(self.friend[i].get_current_position()[0], self.friend[i].get_current_position()[1], a, b, c)
                    if distance < self.ctrld_robot.robot_r * 3:
                        x = (-self.friend[i].get_current_position()[1] * b + (b**2 / a) * self.friend[i].get_current_position()[0] - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.ctrld_robot.get_current_position()[0] < x < goal_pos_x \
                            or self.ctrld_robot.get_current_position()[0] > x > goal_pos_x) \
                            and (self.ctrld_robot.get_current_position()[1] < y < goal_pos_y \
                            or self.ctrld_robot.get_current_position()[1] > y > goal_pos_y):

                            return True, x, y, self.friend[i].get_current_position()[0] , self.friend[i].get_current_position()[1], distance

            for j in range(len(self.enemy)):
                distance = functions.distance_of_a_point_and_a_straight_line(self.enemy[j].get_current_position()[0], self.enemy[j].get_current_position()[1], a, b, c)
                if distance < self.ctrld_robot.robot_r * 3:
                        x = (-self.enemy[j].get_current_position()[1] * b + (b**2 / a) * self.enemy[j].get_current_position()[0] - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.ctrld_robot.get_current_position()[0] < x < goal_pos_x \
                            or self.ctrld_robot.get_current_position()[0] > x > goal_pos_x) \
                            and (self.ctrld_robot.get_current_position()[1] < y < goal_pos_y \
                            or self.ctrld_robot.get_current_position()[1] > y > goal_pos_y):

                            return True, x, y, self.enemy[j].get_current_position()[0] , self.enemy[j].get_current_position()[1], distance

            distance = functions.distance_of_a_point_and_a_straight_line(self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], a, b, c)

            if distance < self.ctrld_robot.robot_r * 2:
                        x = (-self.ball_params.get_current_position()[1] * b + (b**2 / a) * self.ball_params.get_current_position()[0] - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.ctrld_robot.get_current_position()[0] < x < goal_pos_x \
                            or self.ctrld_robot.get_current_position()[0] > x > goal_pos_x) \
                            and (self.ctrld_robot.get_current_position()[1] < y < goal_pos_y \
                            or self.ctrld_robot.get_current_position()[1] > y > goal_pos_y):

                            return True, x, y, self.ball_params.get_current_position()[0] , self.ball_params.get_current_position()[1], distance

        return False, 0, 0, 0, 0, 0

    def avoid_penalty_area(self, goal_pos_x, goal_pos_y):
        a, b, c = functions.line_parameters(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], goal_pos_x, goal_pos_y)
        crossing_flag_r1 = False
        crossing_flag_r2 = False
        crossing_flag_r3 = False
        crossing_flag_l1 = False
        crossing_flag_l2 = False
        crossing_flag_l3 = False

        if a != 0 and b != 0:
            if -6.0 < ((-1.2 * b - c) / a) < -4.8 \
                and (self.ctrld_robot.get_current_position()[0] < ((-1.2 * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-1.2 * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_r1 = True

            if 4.8 < ((-1.2 * b - c) / a) < 6.0 \
                and (self.ctrld_robot.get_current_position()[0] < ((-1.2 * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-1.2 * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_l1 = True


            if -1.2 < ((4.8 * a - c) / b) < 1.2 \
                and (self.ctrld_robot.get_current_position()[1] < ((4.8 * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((4.8 * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_r2 = True

            if -1.2 < ((-4.8 * a - c) / b) < 1.2 \
                and (self.ctrld_robot.get_current_position()[1] < ((-4.8 * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-4.8 * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_l2 = True


            if -6.0 < ((1.2 * b - c) / a) < -4.8 \
                and (self.ctrld_robot.get_current_position()[0] < ((1.2 * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((1.2 * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_r3 = True

            if 4.8 < ((1.2 * b - c) / a) < 6.0 \
                and (self.ctrld_robot.get_current_position()[0] < ((1.2 * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((1.2 * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_l3 = True


            if (crossing_flag_r1 == True) and (crossing_flag_r2 == True):
                goal_pos_x = -4.5
                goal_pos_y = 1.5
                return goal_pos_x, goal_pos_y

            if (crossing_flag_r2 == True) and (crossing_flag_r3 == True):
                goal_pos_x = -4.5
                goal_pos_y = -1.5
                return goal_pos_x, goal_pos_y

            if (crossing_flag_r1 == True) and (crossing_flag_r3 == True):
                if self.ctrld_robot.get_current_position()[1] > 0:    
                    goal_pos_x = -4.5
                    goal_pos_y = 1.5
                    return goal_pos_x, goal_pos_y
                else:
                    goal_pos_x = -4.5
                    goal_pos_y = -1.5
                    return goal_pos_x, goal_pos_y

            if (crossing_flag_l1 == True) and (crossing_flag_l2 == True):
                goal_pos_x = 4.5
                goal_pos_y = 1.5
                return goal_pos_x, goal_pos_y

            if (crossing_flag_l2 == True) and (crossing_flag_l3 == True):
                goal_pos_x = 4.5
                goal_pos_y = -1.5
                return goal_pos_x, goal_pos_y

            if (crossing_flag_l1 == True) and (crossing_flag_l3 == True):
                if self.ctrld_robot.get_current_position()[1] > 0:    
                    goal_pos_x = 4.5
                    goal_pos_y = 1.5
                    return goal_pos_x, goal_pos_y
                else:
                    goal_pos_x = 4.5
                    goal_pos_y = -1.5
                    return goal_pos_x, goal_pos_y

        return goal_pos_x, goal_pos_y

    def avoid_goal(self, goal_pos_x, goal_pos_y):
        a, b, c = functions.line_parameters(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], goal_pos_x, goal_pos_y)
        crossing_flag_r1 = False
        crossing_flag_r2 = False
        crossing_flag_r3 = False
        crossing_flag_r4 = False
        crossing_flag_l1 = False
        crossing_flag_l2 = False
        crossing_flag_l3 = False
        crossing_flag_l4 = False

        if a != 0 and b != 0:
            if -6.3 < ((-0.72 * b - c) / a) < -5.9 \
                and (self.ctrld_robot.get_current_position()[0] < ((-0.72 * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-0.72 * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_r1 = True

            if 5.9 < ((-0.72 * b - c) / a) < 6.3 \
                and (self.ctrld_robot.get_current_position()[0] < ((-0.72 * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-0.72 * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_l1 = True


            if -0.72 < ((5.9 * a - c) / b) < 0.72 \
                and (self.ctrld_robot.get_current_position()[1] < ((5.9 * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((5.9 * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_r2 = True

            if -0.72 < ((-5.9 * a - c) / b) < 0.72 \
                and (self.ctrld_robot.get_current_position()[1] < ((-5.9 * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-5.9 * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_l2 = True


            if -6.3 < ((0.72 * b - c) / a) < -5.9 \
                and (self.ctrld_robot.get_current_position()[0] < ((0.72 * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((0.72 * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_r3 = True

            if 5.9 < ((0.72 * b - c) / a) < 6.3 \
                and (self.ctrld_robot.get_current_position()[0] < ((0.72 * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((0.72 * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_l3 = True

            if -0.72 < ((6.3 * a - c) / b) < 0.72 \
                and (self.ctrld_robot.get_current_position()[1] < ((6.3 * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((6.3 * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_r4 = True

            if -0.72 < ((-6.3 * a - c) / b) < 0.72 \
                and (self.ctrld_robot.get_current_position()[1] < ((-6.3 * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-6.3 * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_l4 = True



            if (crossing_flag_r1 == True) and (crossing_flag_r2 == True):
                goal_pos_x = -5.7
                goal_pos_y = 0.92
                return goal_pos_x, goal_pos_y

            if (crossing_flag_r1 == True) and (crossing_flag_r4 == True):
                goal_pos_x = -6.5
                goal_pos_y = 0.92
                return goal_pos_x, goal_pos_y

            if (crossing_flag_r2 == True) and (crossing_flag_r3 == True):
                goal_pos_x = -5.7
                goal_pos_y = -0.92
                return goal_pos_x, goal_pos_y

            if (crossing_flag_r3 == True) and (crossing_flag_r4 == True):
                goal_pos_x = -6.5
                goal_pos_y = -0.92
                return goal_pos_x, goal_pos_y
            
            if (crossing_flag_r1 == True) and (crossing_flag_r3 == True):
                if  goal_pos_x >= -6.07 and goal_pos_y >= 0:    
                    goal_pos_x = -5.7
                    goal_pos_y = -1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x < -6.07 and goal_pos_y >= 0:
                    goal_pos_x = -6.5
                    goal_pos_y = -1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x >= -6.07 and goal_pos_y < 0:
                    goal_pos_x = -5.7
                    goal_pos_y = 1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x < -6.07 and goal_pos_y < 0:
                    goal_pos_x = -6.5
                    goal_pos_y = 1.2
                    return goal_pos_x, goal_pos_y

            if (crossing_flag_r2 == True) and (crossing_flag_r4 == True):
                if  goal_pos_x >= -6.07 and goal_pos_y >= 0:    
                    goal_pos_x = -6.5
                    goal_pos_y = 1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x < -6.07 and goal_pos_y >= 0:
                    goal_pos_x = -5.7
                    goal_pos_y = 1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x >= -6.07 and goal_pos_y < 0:
                    goal_pos_x = -6.5
                    goal_pos_y = -1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x < -6.07 and goal_pos_y < 0:
                    goal_pos_x = -5.7
                    goal_pos_y = -1.2
                    return goal_pos_x, goal_pos_y
            
            if (crossing_flag_l1 == True) and (crossing_flag_l2 == True):
                goal_pos_x = 5.7
                goal_pos_y = 0.92
                return goal_pos_x, goal_pos_y

            if (crossing_flag_l1 == True) and (crossing_flag_l4 == True):
                goal_pos_x = 6.5
                goal_pos_y = 0.92
                return goal_pos_x, goal_pos_y

            if (crossing_flag_l2 == True) and (crossing_flag_l3 == True):
                goal_pos_x = 5.7
                goal_pos_y = -0.92
                return goal_pos_x, goal_pos_y

            if (crossing_flag_l3 == True) and (crossing_flag_l4 == True):
                goal_pos_x = 6.5
                goal_pos_y = -0.92
                return goal_pos_x, goal_pos_y

            if (crossing_flag_l1 == True) and (crossing_flag_l3 == True):
                if  goal_pos_x <= 6.07 and goal_pos_y >= 0:    
                    goal_pos_x = 5.7
                    goal_pos_y = -1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x > 6.07 and goal_pos_y >= 0:
                    goal_pos_x = 6.5
                    goal_pos_y = -1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x <= 6.07 and goal_pos_y < 0:
                    goal_pos_x = 5.7
                    goal_pos_y = 1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x > 6.07 and goal_pos_y < 0:
                    goal_pos_x = 6.5
                    goal_pos_y = 1.2
                    return goal_pos_x, goal_pos_y

            if (crossing_flag_l2 == True) and (crossing_flag_l4 == True):
                if  goal_pos_x <= 6.07 and goal_pos_y >= 0:    
                    goal_pos_x = 6.5
                    goal_pos_y = 1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x > 6.07 and goal_pos_y >= 0:
                    goal_pos_x = 5.7
                    goal_pos_y = 1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x <= 6.07 and goal_pos_y < 0:
                    goal_pos_x = 6.5
                    goal_pos_y = -1.2
                    return goal_pos_x, goal_pos_y
                elif goal_pos_x > 6.07 and goal_pos_y < 0:
                    goal_pos_x = 5.7
                    goal_pos_y = -1.2
                    return goal_pos_x, goal_pos_y
        return goal_pos_x, goal_pos_y

    def get_sub_goal(self, x, y, obstacle_x, obstacle_y, distance):
        if distance != 0:
            sub_goal_x = (-5 * self.ctrld_robot.robot_r * obstacle_x + (distance + 5 * self.ctrld_robot.robot_r) * x) / distance
            sub_goal_y = (-5 * self.ctrld_robot.robot_r * obstacle_y + (distance + 5 * self.ctrld_robot.robot_r) * y) / distance
            return sub_goal_x, sub_goal_y
        else :
            return 0, 0

    def pass_plan(self, goal_pos_x, goal_pos_y):
        self.recursion_count += 1
        collision = self.collision_detection(goal_pos_x, goal_pos_y)
        if collision[0] and self.recursion_count < self.recursion_max:
            goal_pos_x, goal_pos_y = self.get_sub_goal(collision[1], collision[2], collision[3], collision[4], collision[5])
            self.pass_plan(goal_pos_x, goal_pos_y)
        return goal_pos_x, goal_pos_y

    def set_pid_callback(self, msg):
        self.Kpv = msg.Kpv
        self.Kpr = msg.Kpr
        self.Kdv = msg.Kdv
        self.Kdr = msg.Kdr

        print self.Kpv, self.Kpr, self.Kdv, self.Kdr

    def pid_linear(self, goal_pos_x, goal_pos_y, goal_pos_theta):

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
            
            self.next_pos_x, self.next_pos_y = self.avoid_penalty_area(goal_pos_x, goal_pos_y)
            self.next_pos_x, self.next_pos_y = self.avoid_goal(self.next_pos_x, self.next_pos_y)
            self.next_pos_x, self.next_pos_y = self.pass_plan(self.next_pos_x, self.next_pos_y)
            """
            if goal_pos_x != next_pos_x or goal_pos_y != next_pos_y:
                goal_pos_x = next_pos_x
                goal_pos_y = next_pos_y
            """


        d_x = self.next_pos_x - self.ctrld_robot.get_current_position()[0]
        d_y = self.next_pos_y - self.ctrld_robot.get_current_position()[1]
        d_theta = goal_pos_theta - self.ctrld_robot.get_current_orientation()
        if d_theta < 0 and abs(d_theta) > np.pi:
            d_theta = goal_pos_theta - (self.ctrld_robot.get_current_orientation() - 2 * np.pi)

        if d_theta > 0 and abs(d_theta) > np.pi:
            d_theta = (goal_pos_theta - 2 * np.pi) - self.ctrld_robot.get_current_orientation()


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

        self.cmd.vel_surge = Vx*math.cos(self.ctrld_robot.get_current_orientation())+Vy*math.sin(self.ctrld_robot.get_current_orientation())
        self.cmd.vel_sway = -Vx*math.sin(self.ctrld_robot.get_current_orientation())+Vy*math.cos(self.ctrld_robot.get_current_orientation())
        self.cmd.omega = Vr
        self.command_pub.publish(self.cmd)

    def pid_circle(self, center_x, center_y, x, y, theta):
        self.Kpv = 2
        self.Kpr = 7
        self.Kdr = 4
        d_x = center_x - self.ctrld_robot.get_current_position()[0]
        d_y = center_y - self.ctrld_robot.get_current_position()[1]
        d_theta = theta - self.ctrld_robot.get_current_orientation()
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

            self.cmd.vel_surge=(Vx + Vx_tangent)*math.cos(self.ctrld_robot.get_current_orientation())+(Vy + Vy_tangent)*math.sin(self.ctrld_robot.get_current_orientation())
            self.cmd.vel_sway=-(Vx + Vx_tangent)*math.sin(self.ctrld_robot.get_current_orientation())+(Vy + Vy_tangent)*math.cos(self.ctrld_robot.get_current_orientation())
            self.cmd.omega=Vr
            self.command_pub.publish(self.cmd)

    def replan_timerCallback(self, event):
        self.goal_pos_init_flag = True


class RobotStatus:
    def __init__(self, pid, ctrld_robot):
        self.robot_status = "none"

        self.pid = pid

        self.ctrld_robot = ctrld_robot
        self.ctrld_robot.set_future_position(0., 0., 0.)

    def status_callback(self, msg):
        if msg.status == "move_linear" and (self.ctrld_robot.get_future_position()[0] != msg.pid_goal_pos_x or self.ctrld_robot.get_future_position()[1] != msg.pid_goal_pos_y or self.ctrld_robot.get_future_orientation() != msg.pid_goal_theta):
            self.pid.goal_pos_init_flag = True
        self.robot_status = msg.status
        self.ctrld_robot.set_future_position(
            msg.pid_goal_pos_x,
            msg.pid_goal_pos_y,
            msg.pid_goal_theta)

        self.pid.pid_circle_center_x = msg.pid_circle_center_x
        self.pid.pid_circle_center_y = msg.pid_circle_center_y
        self.ctrld_robot.set_pass_target_position(msg.pass_target_pos_x, msg.pass_target_pos_y)

class RobotKick:
    def __init__(self, ball_params, ctrld_robot, pid, cmd, status, command_pub):
        self.kick_power_x = 10
        self.kick_power_z = 0

        self.ball_params = ball_params
        self.ctrld_robot = ctrld_robot
        self.pid = pid
        self.status = status
        self.cmd = cmd
        self.command_pub = command_pub
        self.dispersion1 = [10] * 1
        self.dispersion2 = [10] * 1
        self.rot_dispersion = [10] * 1

        self.access_threshold1 = 0.1
        self.access_threshold2 = 0.5
        self.feint_threshold1 = 0.15
        self.feint_threshold2 = 1.0 
        self.rot_access_threshold = 0.015
        self.pass_stage = 0

        self.const = 1.5*2

        self.ball_pos_x_array = np.array([0.0]*50)
        self.ball_pos_y_array = np.array([0.0]*50)
        self.reach_flag = False
        self.ball_pos_count = 0

        self.plot_x = np.arange(-5.0,5.0, 0.01)
        self.plot_y = np.arange(-5.0,5.0, 0.01)
        self.fig, self.ax = plt.subplots(1, 1)
        # 初期化的に一度plotしなければならない
        # そのときplotしたオブジェクトを受け取る受け取る必要がある．
        # listが返ってくるので，注意
        self.lines1, = self.ax.plot(self.ball_pos_x_array, self.ball_pos_y_array)
        self.lines2, = self.ax.plot(self.plot_x, self.plot_y)
        self.lines3, = self.ax.plot(self.plot_x, self.plot_y)
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

    def kick_x(self):
        area = 0.5
        if math.sqrt((self.ball_params.get_current_position()[0] - self.ctrld_robot.get_current_position()[0])**2 + (self.ball_params.get_current_position()[1] - self.ctrld_robot.get_current_position()[1])**2) > self.ctrld_robot.robot_r + area:
            self.cmd.vel_surge = 0
            self.cmd.vel_sway = 0
            self.cmd.omega = 0
            self.cmd.kick_speed_x = 0
            self.command_pub.publish(self.cmd)
            self.status.robot_status = "None"
            self.pass_stage = 0
            self.dispersion1 = [10] * 1
            self.dispersion2 = [10] * 1
            return

        self.cmd.kick_speed_x = self.kick_power_x
        self.pid.pid_linear(self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], self.pose_theta)
        #self.cmd.vel_surge = 3
        self.command_pub.publish(self.cmd)

    def pass_ball(self, target_x, target_y):
        distance = math.sqrt((target_x - self.ball_params.get_current_position()[0])**2 + (target_y - self.ball_params.get_current_position()[1])**2)
        if distance != 0:
            #print self.pass_stage
            if self.pass_stage == 0:
                pose_x = (- 0.3 * target_x + (0.3 + distance) * self.ball_params.get_current_position()[0]) / distance
                pose_y = (- 0.3 * target_y + (0.3 + distance) * self.ball_params.get_current_position()[1]) / distance
                pose_theta = math.atan2( (target_y - self.ctrld_robot.get_current_position()[1]) , (target_x - self.ctrld_robot.get_current_position()[0]) )

                a, b, c = functions.line_parameters(self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], target_x, target_y)

                self.dispersion1.append(
                        functions.distance_of_a_point_and_a_straight_line(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], a, b, c))
                        #(pose_x - self.ctrld_robot.get_current_position()[0])**2 \
                        #+ (pose_y - self.ctrld_robot.get_current_position()[1])**2)
                del self.dispersion1[0]

                self.dispersion2.append(
                        (pose_x - self.ctrld_robot.get_current_position()[0])**2 \
                        + (pose_y - self.ctrld_robot.get_current_position()[1])**2)
                del self.dispersion2[0]

                dispersion_average1 = sum(self.dispersion1)/len(self.dispersion1)
                dispersion_average2 = sum(self.dispersion2)/len(self.dispersion2)

                if dispersion_average1 > self.feint_threshold1 or dispersion_average2 > self.feint_threshold2:
                    pose_theta += np.pi/3.

                self.rot_dispersion.append((pose_theta - self.ctrld_robot.get_current_orientation())**2)
                del self.rot_dispersion[0]
                rot_dispersion_average = sum(self.rot_dispersion)/len(self.rot_dispersion)

                # print("{} {}".format(dispersion_average, rot_dispersion_average))

                if dispersion_average1 < self.access_threshold1 and dispersion_average2 < self.access_threshold2 and rot_dispersion_average < self.rot_access_threshold:
                    self.kick_power_x = math.sqrt(distance) * self.const
                    self.pose_theta = pose_theta
                    self.status.robot_status = "kick"
                    #self.pass_stage = 1
            """
            if self.pass_stage == 1:
                self.kick_power_x = math.sqrt(distance) * self.const
                self.pose_theta = pose_theta
                self.status.robot_status = "kick"
            """
            
            self.pid.pid_linear(pose_x, pose_y, pose_theta)

    def kick_z(self):
        pass

    def reg1dim(self, x, y):
        x = np.clip(x,-5,5)
        y = np.clip(y,-5,5)
        n = len(x)
        a = ((np.dot(x, y)- y.sum() * x.sum()/n) / ((x ** 2).sum() - x.sum()**2 / n))
        b = (y.sum() - a * x.sum())/n
        a = np.clip(a,-1.0e+308,1.0e+308)
        b = np.clip(b,-1.0e+308,1.0e+308)
        return a, b

    def recieve_ball(self, target_x, target_y):

        self.reach_flag = True
        #目標点まで移動
        if self.reach_flag == False:
            pose_theta = math.atan2( (self.ball_params.get_current_position()[1] - target_y) , (self.ball_params.get_current_position()[0] - target_x) )
            self.pid.pid_linear(target_x, target_y, pose_theta)
            distance = math.sqrt((target_x - self.ctrld_robot.get_current_position()[0])**2 + (target_y - self.ctrld_robot.get_current_position()[1])**2)
            if distance < 0.1:
                self.reach_flag = False
                #print("reach")
            #else:
                #print(distance)

        #50カウント毎の座標を取得
        if self.ball_pos_count < 50:
            self.ball_pos_x_array[self.ball_pos_count] = self.ball_params.get_current_position()[0]
            self.ball_pos_y_array[self.ball_pos_count] = self.ball_params.get_current_position()[1]
            self.ball_pos_count+=1
        else:
            """ for i in range(0,50):
                self.ball_pos_x_array[ball_pos_count] = 0
                self.ball_pos_y_array[ball_pos_count] = 0
            ball_pos_count = 0 """
            self.ball_pos_x_array = np.roll(self.ball_pos_x_array,-1)
            self.ball_pos_y_array = np.roll(self.ball_pos_y_array,-1)
            self.ball_pos_x_array[self.ball_pos_count-1] = self.ball_params.get_current_position()[0]
            self.ball_pos_y_array[self.ball_pos_count-1] = self.ball_params.get_current_position()[1]

        if self.ball_pos_count % 1 == 0:
            a, b = self.reg1dim(self.ball_pos_x_array, self.ball_pos_y_array)
            # 本来のパスゴール地点と実際の直線Lとの距離計算
            d = (abs(a*target_x-target_y+b))/((a**2+1)**(1/2)) # ヘッセの公式で距離計算
            # 交点H(hx, hy) の座標計算
            hx = (a*(target_y-b)+target_x)/(a**2+1)
            hy = a*(a*(target_y-b)+target_x)/(a**2+1)+b

            if d < 2:
                pose_theta = math.atan2( (self.ball_params.get_current_position()[1] - hy) , (self.ball_params.get_current_position()[0] - hx) )
                self.pid.pid_linear(hx, hy, pose_theta)
            else:
                pose_theta = math.atan2( (self.ball_params.get_current_position()[1] - target_y) , (self.ball_params.get_current_position()[0] - target_x) )
                self.pid.pid_linear(target_x, target_y, pose_theta)

            # 垂線テキスト座標
            dx_center = (target_x + hx) / 2
            dy_center = (target_y + hy) / 2
            # plt.axis('scaled')
            #plt.plot([target_x, hx],[target_y, hy], color='green', linestyle='--', zorder=0)

            self.plot_y = a * self.plot_x + b
            self.lines1.set_data(self.ball_pos_x_array, self.ball_pos_y_array)
            self.lines2.set_data(self.plot_x, self.plot_y)
            self.lines3.set_data([target_x, hx], [target_y, hy])
            # plt.pause(.01)


class RobotStrategy:
    def __init__(self):
        pass

    def attack_startegy(self):
        pass
