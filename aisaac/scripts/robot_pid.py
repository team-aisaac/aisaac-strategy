#!/usr/bin/env  python
# coding:utf-8
import math
import functions
import config
import numpy as np
import rospy

ROBOT_LOOP_RATE = config.ROBOT_LOOP_RATE

class RobotPid(object):    
    def __init__(self, robot_id, objects, cmd):
        self.robot_id = int(robot_id)
        self.ctrld_robot = objects.robot[int(robot_id)]
        self.friend = objects.robot
        self.enemy = objects.enemy
        self.ball_params = objects.ball
        self.objects = objects
        self.cmd = cmd

        self.goal_pos_init_flag = True

        self.pid_circle_center_x = 0
        self.pid_circle_center_y = 0

        self.recursion_max = 10
        self.recursion_count = 0

        self.last_loop_time = rospy.Time.now()
        self.dt = 0

        # self.Kpv = 3.615645812128088
        # self.Kpr = 3.0
        # self.Kdv = 1.9759837181620452
        # self.Kdr = 3.0
        # self.Kpv = 6.112
        # self.Kpr = 3.0
        # self.Kdv = 5.437
        # self.Kdr = 2.0
        # # param_dict = {'Kdv': 4.210412653218034, 'Kpv': 3.9577989929770307, 'Kpr': 7.492985949699556, 'Kdr': 5.743020391472334}
        # self.Kpr = param_dict['Kpr']
        # self.Kpv = param_dict['Kpv']
        # self.Kdv = param_dict['Kdv']
        # self.Kdr = param_dict['Kdr']


        # self.Kpv = 3.5
        # self.Kpr = 3.5
        # self.Kdv = 5.0
        # self.Kdr = 5.0

        # 実機
        # self.Kpv = 4.5
        # self.Kpr = 2.0
        # self.Kdv = 1.5
        # self.Kdr = 1.0

        self.Kpv = 2.2
        self.Kpr = 1.7
        self.Kdv = 1.0
        self.Kdr = 0.8

        # 壁用
        # self.Kpv = 3.0
        # self.Kpr = 6.0
        # self.Kdv = 3.0
        # self.Kdr = 4.0
        
        # self.Kpv = 3.8
        # self.Kpr = 4
        # self.Kdv = 12
        # self.Kdr = 3

    def collision_detection(self, goal_pos_x, goal_pos_y):
        a, b, c = functions.line_parameters(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], goal_pos_x, goal_pos_y)
        if a != 0 and b != 0:
            for i in self.objects.get_active_robot_ids():
                if i != self.robot_id:
                    robot = self.objects.get_robot_by_id(i)
                    if robot is None:
                        continue
                    distance = functions.distance_of_a_point_and_a_straight_line(robot.get_current_position()[0], robot.get_current_position()[1], a, b, c)
                    if distance < self.ctrld_robot.size_r * 2:
                        x = (-robot.get_current_position()[1] * b + (b**2 / a) * robot.get_current_position()[0] - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.ctrld_robot.get_current_position()[0] < x < goal_pos_x \
                            or self.ctrld_robot.get_current_position()[0] > x > goal_pos_x) \
                            and (self.ctrld_robot.get_current_position()[1] < y < goal_pos_y \
                            or self.ctrld_robot.get_current_position()[1] > y > goal_pos_y):

                            return True, x, y, robot.get_current_position()[0] , robot.get_current_position()[1], distance

            for j in self.objects.get_active_enemy_ids():
                enemy = self.objects.get_enemy_by_id(j)
                if enemy is None:
                    continue
                distance = functions.distance_of_a_point_and_a_straight_line(enemy.get_current_position()[0], enemy.get_current_position()[1], a, b, c)
                if distance < self.ctrld_robot.size_r * 2:
                        x = (-enemy.get_current_position()[1] * b + (b**2 / a) * enemy.get_current_position()[0] - c) / (a + b**2 / a)
                        y = (-a * x -c) / b
                        if (self.ctrld_robot.get_current_position()[0] < x < goal_pos_x \
                            or self.ctrld_robot.get_current_position()[0] > x > goal_pos_x) \
                            and (self.ctrld_robot.get_current_position()[1] < y < goal_pos_y \
                            or self.ctrld_robot.get_current_position()[1] > y > goal_pos_y):

                            return True, x, y, enemy.get_current_position()[0] , enemy.get_current_position()[1], distance

            distance = functions.distance_of_a_point_and_a_straight_line(self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], a, b, c)

            if distance < self.ctrld_robot.size_r * 1:
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
        goal_place_l = [-4.8, -6.0, 1.2, -1.2]
        goal_place_r = [6.0, 4.8, 1.2, -1.2]
        crossing_flag_l1 = False
        crossing_flag_l2 = False
        crossing_flag_l3 = False
        crossing_flag_r1 = False
        crossing_flag_r2 = False
        crossing_flag_r3 = False
        sub_goal_l = [[-4.5, 1.5],
                      [-4.5, -1.5]]
        sub_goal_r = [[4.5, 1.5],
                      [4.5, -1.5]]

        if a != 0 and b != 0:
            if goal_place_l[1] < ((-goal_place_l[2] * b - c) / a) < goal_place_l[0] \
                and (self.ctrld_robot.get_current_position()[0] < ((-goal_place_l[2] * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-goal_place_l[2] * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_l1 = True

            if goal_place_r[1] < ((-goal_place_r[2] * b - c) / a) < goal_place_r[0] \
                and (self.ctrld_robot.get_current_position()[0] < ((-goal_place_r[2] * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-goal_place_r[2] * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_r1 = True


            if goal_place_l[3] < ((-goal_place_l[0] * a - c) / b) < goal_place_l[2] \
                and (self.ctrld_robot.get_current_position()[1] < ((-goal_place_l[0] * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-goal_place_l[0] * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_l2 = True

            if goal_place_r[3] < ((-goal_place_r[1] * a - c) / b) < goal_place_r[2] \
                and (self.ctrld_robot.get_current_position()[1] < ((-goal_place_r[1] * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-goal_place_r[1] * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_r2 = True


            if goal_place_l[1] < ((-goal_place_l[3] * b - c) / a) < goal_place_l[0] \
                and (self.ctrld_robot.get_current_position()[0] < ((-goal_place_l[3] * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-goal_place_l[3] * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_l3 = True

            if goal_place_r[1] < ((-goal_place_r[3] * b - c) / a) < goal_place_r[0] \
                and (self.ctrld_robot.get_current_position()[0] < ((-goal_place_r[3] * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-goal_place_r[3] * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_r3 = True


            if (crossing_flag_l1 == True) and (crossing_flag_l2 == True):
                return sub_goal_l[0][0], sub_goal_l[0][1]

            elif (crossing_flag_l2 == True) and (crossing_flag_l3 == True):
                return sub_goal_l[1][0], sub_goal_l[1][1]

            elif (crossing_flag_l1 == True) and (crossing_flag_l3 == True):
                if self.ctrld_robot.get_current_position()[1] > 0:    
                    return sub_goal_l[0][0], sub_goal_l[0][1]
                else:
                    return sub_goal_l[1][0], sub_goal_l[1][1]

            if (crossing_flag_r1 == True) and (crossing_flag_r2 == True):
                return sub_goal_r[0][0], sub_goal_r[0][1]

            elif (crossing_flag_r2 == True) and (crossing_flag_r3 == True):
                return sub_goal_r[1][0], sub_goal_r[1][1]

            elif (crossing_flag_r1 == True) and (crossing_flag_r3 == True):
                if self.ctrld_robot.get_current_position()[1] > 0:    
                    return sub_goal_r[0][0], sub_goal_r[0][1]
                else:
                    return sub_goal_r[1][0], sub_goal_r[1][1]

        return goal_pos_x, goal_pos_y

    def avoid_goal(self, goal_pos_x, goal_pos_y):
        a, b, c = functions.line_parameters(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], goal_pos_x, goal_pos_y)
        goal_place_l = [-5.9, -6.3, 0.72, -0.72]
        goal_place_r = [6.3, 5.9, 0.72, -0.72]
        crossing_flag_l1 = False
        crossing_flag_l2 = False
        crossing_flag_l3 = False
        crossing_flag_l4 = False
        crossing_flag_r1 = False
        crossing_flag_r2 = False
        crossing_flag_r3 = False
        crossing_flag_r4 = False
        within_goal_flag_l = False
        within_goal_flag_r = False
        sub_goal_l = [[-5.7, 0.92],
                      [-5.7, -0.92],
                      [-6.5, -0.92],
                      [-6.5, 0.92]]
        sub_goal_r = [[5.7, 0.92],
                      [5.7, -0.92],
                      [6.5, -0.92],
                      [6.5, 0.92]]

        if a != 0 and b != 0:
            if goal_place_l[1] < ((-goal_place_l[2] * b - c) / a) < goal_place_l[0] \
                and (self.ctrld_robot.get_current_position()[0] < ((-goal_place_l[2] * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-goal_place_l[2] * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_l1 = True

            if goal_place_r[1] < ((-goal_place_r[2] * b - c) / a) < goal_place_r[0] \
                and (self.ctrld_robot.get_current_position()[0] < ((-goal_place_r[2] * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-goal_place_r[2] * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_r1 = True


            if goal_place_l[3] < ((-goal_place_l[0] * a - c) / b) < goal_place_l[2] \
                and (self.ctrld_robot.get_current_position()[1] < ((-goal_place_l[0] * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-goal_place_l[0] * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_l2 = True

            if goal_place_r[3] < ((-goal_place_r[1] * a - c) / b) < goal_place_r[2] \
                and (self.ctrld_robot.get_current_position()[1] < ((-goal_place_r[1] * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-goal_place_r[1] * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_r2 = True


            if goal_place_l[1] < ((-goal_place_l[3] * b - c) / a) < goal_place_l[0] \
                and (self.ctrld_robot.get_current_position()[0] < ((-goal_place_l[3] * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-goal_place_l[3] * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_l3 = True

            if goal_place_r[1] < ((-goal_place_r[3] * b - c) / a) < goal_place_r[0] \
                and (self.ctrld_robot.get_current_position()[0] < ((-goal_place_r[3] * b - c) / a) < goal_pos_x \
                or goal_pos_x < ((-goal_place_r[3] * b - c) / a) < self.ctrld_robot.get_current_position()[0]):

                crossing_flag_r3 = True

            if goal_place_l[3] < ((-goal_place_l[1] * a - c) / b) < goal_place_l[2] \
                and (self.ctrld_robot.get_current_position()[1] < ((-goal_place_l[1] * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-goal_place_l[1] * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_l4 = True

            if goal_place_r[3] < ((-goal_place_r[0] * a - c) / b) < goal_place_r[2] \
                and (self.ctrld_robot.get_current_position()[1] < ((-goal_place_r[0] * a - c) / b) < goal_pos_y \
                or goal_pos_y < ((-goal_place_r[0] * a - c) / b) < self.ctrld_robot.get_current_position()[1]):

                crossing_flag_r4 = True

            if goal_place_l[1] < self.ctrld_robot.get_current_position()[0] < goal_place_l[0] \
                and goal_place_l[3] < self.ctrld_robot.get_current_position()[1] < goal_place_l[2]:

                within_goal_flag_l = True

            if goal_place_r[1] < self.ctrld_robot.get_current_position()[0] < goal_place_r[0] \
                and goal_place_r[3] < self.ctrld_robot.get_current_position()[1] < goal_place_r[2]:

                within_goal_flag_r = True



            if (crossing_flag_l1 == True) and (crossing_flag_l2 == True):
                return sub_goal_l[0][0], sub_goal_l[0][1]

            elif (crossing_flag_l1 == True) and (crossing_flag_l4 == True):
                return sub_goal_l[3][0], sub_goal_l[3][1]

            elif (crossing_flag_l2 == True) and (crossing_flag_l3 == True):
                return sub_goal_l[1][0], sub_goal_l[1][1]

            elif (crossing_flag_l3 == True) and (crossing_flag_l4 == True):
                return sub_goal_l[2][0], sub_goal_l[2][1]
            
            elif (crossing_flag_l1 == True) and (crossing_flag_l3 == True):
                if  goal_pos_x >= (goal_place_l[0] + goal_place_l[1]) / 2. and goal_pos_y >= 0:    
                    return sub_goal_l[1][0], sub_goal_l[1][1] - 0.3
                elif goal_pos_x < (goal_place_l[0] + goal_place_l[1]) / 2. and goal_pos_y >= 0:
                    return sub_goal_l[2][0], sub_goal_l[2][1] - 0.3
                elif goal_pos_x >= (goal_place_l[0] + goal_place_l[1]) / 2. and goal_pos_y < 0:
                    return sub_goal_l[0][0], sub_goal_l[0][1] + 0.3
                elif goal_pos_x < (goal_place_l[0] + goal_place_l[1]) / 2. and goal_pos_y < 0:
                    return sub_goal_l[3][0], sub_goal_l[3][1] + 0.3

            elif (crossing_flag_l2 == True) and (crossing_flag_l4 == True):
                if  goal_pos_x >= (goal_place_l[0] + goal_place_l[1]) / 2. and goal_pos_y >= 0:    
                    return sub_goal_l[3][0], sub_goal_l[3][1] + 0.3
                elif goal_pos_x < (goal_place_l[0] + goal_place_l[1]) / 2. and goal_pos_y >= 0:
                    return sub_goal_l[0][0], sub_goal_l[0][1] + 0.3
                elif goal_pos_x >= (goal_place_l[0] + goal_place_l[1]) / 2. and goal_pos_y < 0:
                    return sub_goal_l[2][0], sub_goal_l[2][1] - 0.3
                elif goal_pos_x < (goal_place_l[0] + goal_place_l[1]) / 2. and goal_pos_y < 0:
                    return sub_goal_l[1][0], sub_goal_l[1][1] - 0.3

            elif (within_goal_flag_l == True) and ((crossing_flag_l1 == True) or (crossing_flag_l3 == True) or (crossing_flag_l4 == True)):
                return sub_goal_l[0][0], 0
            
            if (crossing_flag_r1 == True) and (crossing_flag_r2 == True):
                return sub_goal_r[0][0], sub_goal_r[0][1]

            elif (crossing_flag_r1 == True) and (crossing_flag_r4 == True):
                return sub_goal_r[3][0], sub_goal_r[3][1]

            elif (crossing_flag_r2 == True) and (crossing_flag_r3 == True):
                return sub_goal_r[1][0], sub_goal_r[1][1]

            elif (crossing_flag_r3 == True) and (crossing_flag_r4 == True):
                return sub_goal_r[2][0], sub_goal_r[2][1]

            elif (crossing_flag_r1 == True) and (crossing_flag_r3 == True):
                if  goal_pos_x <= (goal_place_r[0] + goal_place_r[1]) / 2. and goal_pos_y >= 0:    
                    return sub_goal_r[1][0], sub_goal_r[1][1] - 0.3
                elif goal_pos_x > (goal_place_r[0] + goal_place_r[1]) / 2. and goal_pos_y >= 0:
                    return sub_goal_r[2][0], sub_goal_r[2][1] - 0.3
                elif goal_pos_x <= (goal_place_r[0] + goal_place_r[1]) / 2. and goal_pos_y < 0:
                    return sub_goal_r[0][0], sub_goal_r[0][1] + 0.3
                elif goal_pos_x > (goal_place_r[0] + goal_place_r[1]) / 2. and goal_pos_y < 0:
                    return sub_goal_r[3][0], sub_goal_r[3][1] + 0.3

            elif (crossing_flag_r2 == True) and (crossing_flag_r4 == True):
                if  goal_pos_x <= (goal_place_r[0] + goal_place_r[1]) / 2. and goal_pos_y >= 0:    
                    return sub_goal_r[3][0], sub_goal_r[3][1] + 0.3
                elif goal_pos_x > (goal_place_r[0] + goal_place_r[1]) / 2. and goal_pos_y >= 0:
                    return sub_goal_r[0][0], sub_goal_r[0][1] + 0.3
                elif goal_pos_x <= (goal_place_r[0] + goal_place_r[1]) / 2. and goal_pos_y < 0:
                    return sub_goal_r[2][0], sub_goal_r[2][1] - 0.3
                elif goal_pos_x > (goal_place_r[0] + goal_place_r[1]) / 2. and goal_pos_y < 0:
                    return sub_goal_r[1][0], sub_goal_r[1][1] - 0.3

            elif (within_goal_flag_r == True) and ((crossing_flag_r1 == True) or (crossing_flag_r3 == True) or (crossing_flag_r4 == True)):
                return sub_goal_r[0][0], 0

        return goal_pos_x, goal_pos_y

    def get_sub_goal(self, x, y, obstacle_x, obstacle_y, distance):
        penalty_area_l = [-4.8, -6.0, 1.2, -1.2]
        penalty_area_r = [6.0, 4.8, 1.2, -1.2]

        if distance != 0:
            sub_goal_x = (-5 * self.ctrld_robot.size_r * obstacle_x + (distance + 5 * self.ctrld_robot.size_r) * x) / distance
            sub_goal_y = (-5 * self.ctrld_robot.size_r * obstacle_y + (distance + 5 * self.ctrld_robot.size_r) * y) / distance

            if (penalty_area_l[1] < sub_goal_x < penalty_area_l[0] \
                and penalty_area_l[3] < sub_goal_y < penalty_area_l[2]) \
                or (penalty_area_r[1] < sub_goal_x < penalty_area_r[0] \
                and penalty_area_r[3] < sub_goal_y < penalty_area_r[2]):

                sub_goal_x = (-6 * self.ctrld_robot.size_r * x + (distance + 6 * self.ctrld_robot.size_r) * obstacle_x) / distance
                sub_goal_y = (-6 * self.ctrld_robot.size_r * y + (distance + 6 * self.ctrld_robot.size_r) * obstacle_y) / distance

            return sub_goal_x, sub_goal_y
        else :
            return 0, 0

    def path_plan(self, goal_pos_x, goal_pos_y):
        self.recursion_count += 1
        collision = self.collision_detection(goal_pos_x, goal_pos_y)
        if collision[0] and self.recursion_count < self.recursion_max:
            goal_pos_x, goal_pos_y = self.get_sub_goal(collision[1], collision[2], collision[3], collision[4], collision[5])
            self.path_plan(goal_pos_x, goal_pos_y)
        return goal_pos_x, goal_pos_y

    def set_pid_callback(self, msg):
        self.Kpv = msg.Kpv
        self.Kpr = msg.Kpr
        self.Kdv = msg.Kdv
        self.Kdr = msg.Kdr

        print self.Kpv, self.Kpr, self.Kdv, self.Kdr

        return 1


    def _clip_penalty_area(self, goal_pos_x, goal_pos_y, offset=0.0):
        """
        Parameters
        ----------
        goal_pos_x, goal_pos_y 省略
        offset: float offsetメートル分ペナルティエリアが各辺に対して広いと仮定して計算する

        Return
        ----------
        goal_pos_x, goal_pos_y: (float, float) 自分-目的地の直線とペナルティエリアの交点の座標を返す
        """

        in_penalty_area = functions.in_penalty_area((goal_pos_x, goal_pos_y), offset=offset)

        if not in_penalty_area:
            return goal_pos_x, goal_pos_y

        penal_area_points = {}
        penal_area_points['friend'] = {
            'left_bot': [-6.0 - offset, -1.2 - offset],
            'right_bot': [-4.8 + offset, -1.2 - offset],
            'right_top': [-4.8 + offset, 1.2 + offset],
            'left_top': [-6.0 - offset, 1.2 + offset],
        }
        penal_area_points['enemy'] = {
            'right_top': [6.0 + offset, 1.2 + offset],
            'left_top': [4.8 - offset, 1.2 + offset],
            'left_bot': [4.8 + offset, -1.2 - offset],
            'right_bot': [6.0 + offset, -1.2 - offset],
        }

        robo_x, robo_y = self.ctrld_robot.get_current_position()
        line_robot_to_goal = \
            functions.line_parameters(robo_x, robo_y, goal_pos_x, goal_pos_y)

        line_penal_dict = {}

        line_penal_dict['friend'] = {}
        line_penal_dict['friend']['front'] = functions.line_parameters_vector_args(
            penal_area_points['friend']['right_bot'],
            penal_area_points['friend']['right_top']
        )
        line_penal_dict['friend']['top'] = functions.line_parameters_vector_args(
            penal_area_points['friend']['left_top'],
            penal_area_points['friend']['right_top'],
        )
        line_penal_dict['friend']['bot'] = functions.line_parameters_vector_args(
            penal_area_points['friend']['left_bot'],
            penal_area_points['friend']['right_bot'],
        )

        line_penal_dict['enemy'] = {}
        line_penal_dict['enemy']['front'] = functions.line_parameters_vector_args(
            penal_area_points['enemy']['left_bot'],
            penal_area_points['enemy']['left_top']
        )
        line_penal_dict['enemy']['top'] = functions.line_parameters_vector_args(
            penal_area_points['enemy']['left_top'],
            penal_area_points['enemy']['right_top']
        )
        line_penal_dict['enemy']['bot'] = functions.line_parameters_vector_args(
            penal_area_points['enemy']['left_bot'],
            penal_area_points['enemy']['right_bot']
        )

        if in_penalty_area == "friend":
            line_penal = line_penal_dict['friend']
        else:
            line_penal = line_penal_dict['enemy']

        clip_pos_xys = []
        for key in line_penal.keys():
            tmp_cross_point = functions.cross_point(line_penal[key], line_robot_to_goal)
            if functions.in_penalty_area(tmp_cross_point, offset=offset + 0.1):
                clip_pos_xys.append(tmp_cross_point)

        sorted_clip_pos_xys = sorted(clip_pos_xys,
                                     key=lambda clip_pos_xy: functions.distance_btw_two_points(
                                         clip_pos_xy,
                                         self.ctrld_robot.get_current_position()))

        return sorted_clip_pos_xys[0]

    def pid_linear(self, goal_pos_x, goal_pos_y, goal_pos_theta, ignore_penalty_area=False, avoid=True):
        """
        Parameters
        ----------
        goal_pos_x: float 目的地のx座標
        goal_pos_y: float 目的地のy座標
        goal_pos_theta: float 目的の角度
        ignore_penalty_area: boolean Trueならペナルティエリアに進入する、Falseなら進入しない
        avoid: boolean Trueなら障害物回避を行う
        """

        """
        self.recursion_count = 0
        next_pos_x, next_pos_y = self.path_plan(goal_pos_x, goal_pos_y)
        if goal_pos_x != next_pos_x or goal_pos_y != next_pos_y:
            self.goal_pos_init_flag = True
            goal_pos_x = next_pos_x
            goal_pos_y = next_pos_y
        """

        self.dt = rospy.Time.now() - self.last_loop_time
        self.last_loop_time = rospy.Time.now()
        #print self.dt.to_sec()

        if self.goal_pos_init_flag == True:
            self.recursion_count = 0

            tmp_x = goal_pos_x
            tmp_y = goal_pos_y


            tmp_x, tmp_y = self.avoid_penalty_area(tmp_x, tmp_y)
            tmp_x, tmp_y = self.avoid_goal(tmp_x, tmp_y)

            if avoid:
                tmp_x, tmp_y = self.path_plan(tmp_x, tmp_y)

            if not ignore_penalty_area:
                tmp_x, tmp_y = self._clip_penalty_area(tmp_x, tmp_y, offset=self.ctrld_robot.size_r)

            self.next_pos_x = tmp_x
            self.next_pos_y = tmp_y
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

        Vx = self.Kpv * self.pid_p_x + self.Kdv * self.pid_d_x / self.dt.to_sec()
        Vy = self.Kpv * self.pid_p_y + self.Kdv * self.pid_d_y / self.dt.to_sec()
        Vr = self.Kpr * self.pid_p_theta + self.Kdr * self.pid_d_theta / self.dt.to_sec()


        max_velocity = rospy.get_param("/robot_max_velocity", default=config.ROBOT_MAX_VELOCITY) # m/s 機体の最高速度
        vel_vector = np.array([Vx, Vy])
        vel_vector_norm = np.linalg.norm(vel_vector)
        if vel_vector_norm > max_velocity:
            vel_vector = vel_vector * max_velocity / vel_vector_norm
            Vx = vel_vector[0]
            Vy = vel_vector[1]

        self.cmd.vel_x = Vx
        self.cmd.vel_y = Vy
        self.cmd.vel_surge = Vx*math.cos(self.ctrld_robot.get_current_orientation())+Vy*math.sin(self.ctrld_robot.get_current_orientation())
        self.cmd.vel_sway = -Vx*math.sin(self.ctrld_robot.get_current_orientation())+Vy*math.cos(self.ctrld_robot.get_current_orientation())
        self.cmd.omega = Vr
        self.cmd.theta = goal_pos_theta

    def pid_circle(self, center_x, center_y, x, y, theta):        
        """
        2019/08/04 当初敵の撹乱などに利用予定だったが作業時間の都合上今は利用していない
        """
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

    def replan_timer_callback(self, event):
        self.goal_pos_init_flag = True
