#!/usr/bin/env  python
# coding:utf-8
import math
import functions
import config
import numpy as np
import rospy

ROBOT_LOOP_RATE = config.ROBOT_LOOP_RATE

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

        # self.Kpv = 2.2
        # self.Kpr = 2.0
        # self.Kdv = 3.0
        # self.Kdr = 1.0
        self.Kpv = 3.0
        self.Kpr = 6.0
        self.Kdv = 3.0
        self.Kdr = 4.0
        # self.Kpv = 3.8
        # self.Kpr = 4
        # self.Kdv = 12
        # self.Kdr = 3

    def collision_detection(self, goal_pos_x, goal_pos_y):
        a, b, c = functions.line_parameters(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], goal_pos_x, goal_pos_y)
        if a != 0 and b != 0:
            for i in range(len(self.friend)):
                if i != self.robot_id:
                    distance = functions.distance_of_a_point_and_a_straight_line(self.friend[i].get_current_position()[0], self.friend[i].get_current_position()[1], a, b, c)
                    if distance < self.ctrld_robot.size_r * 3:
                        x = (-self.friend[i].get_current_position()[1] * b + (b**2 / a) * self.friend[i].get_current_position()[0] - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.ctrld_robot.get_current_position()[0] < x < goal_pos_x \
                            or self.ctrld_robot.get_current_position()[0] > x > goal_pos_x) \
                            and (self.ctrld_robot.get_current_position()[1] < y < goal_pos_y \
                            or self.ctrld_robot.get_current_position()[1] > y > goal_pos_y):

                            return True, x, y, self.friend[i].get_current_position()[0] , self.friend[i].get_current_position()[1], distance

            for j in range(len(self.enemy)):
                distance = functions.distance_of_a_point_and_a_straight_line(self.enemy[j].get_current_position()[0], self.enemy[j].get_current_position()[1], a, b, c)
                if distance < self.ctrld_robot.size_r * 3:
                        x = (-self.enemy[j].get_current_position()[1] * b + (b**2 / a) * self.enemy[j].get_current_position()[0] - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.ctrld_robot.get_current_position()[0] < x < goal_pos_x \
                            or self.ctrld_robot.get_current_position()[0] > x > goal_pos_x) \
                            and (self.ctrld_robot.get_current_position()[1] < y < goal_pos_y \
                            or self.ctrld_robot.get_current_position()[1] > y > goal_pos_y):

                            return True, x, y, self.enemy[j].get_current_position()[0] , self.enemy[j].get_current_position()[1], distance

            distance = functions.distance_of_a_point_and_a_straight_line(self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], a, b, c)

            if distance < self.ctrld_robot.size_r * 2:
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
            sub_goal_x = (-5 * self.ctrld_robot.size_r * obstacle_x + (distance + 5 * self.ctrld_robot.size_r) * x) / distance
            sub_goal_y = (-5 * self.ctrld_robot.size_r * obstacle_y + (distance + 5 * self.ctrld_robot.size_r) * y) / distance
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


