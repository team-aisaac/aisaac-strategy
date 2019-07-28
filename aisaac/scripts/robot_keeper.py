#!/usr/bin/env  python
# coding:utf-8
import numpy as np

class RobotKeeper:
    def __init__(self, robot_id, objects, ball_params, pid, cmd, status):
        self.robot_id = int(robot_id)
        self.ctrld_robot = objects.robot[int(robot_id)]
        self.friend = objects.robot
        self.enemy = objects.enemy
        self.ball_params = ball_params
        self.pid = pid
        self.status = status
        self.cmd = cmd
        self.team_side = "left"

        self.goal_right = [6, -0.620]
        self.goal_left = [6, 0.620]
        self.r_offset = self.ctrld_robot.size_r


    def calc_keeper_position(self, defense_point_x, defense_point_y):
        if self.team_side == "right":
            a1 = -self.ball_params.get_current_position()[1] + defense_point_y
            b1 = defense_point_x - self.ball_params.get_current_position()[0]
            c1 = self.ball_params.get_current_position()[0] * -a1 -self.ball_params.get_current_position()[1] * -b1

            a2 = b1
            b2 = -a1

            if self.ball_params.get_current_position()[1] < 0.:
                c2 =self.goal_right[0] * -a2 - self.goal_right[1] * -b2
            else:
                c2 =self.goal_left[0] * -a2 - self.goal_left[1] * -b2

            if np.sqrt((self.ball_params.get_current_position()[0] - defense_point_x)**2 + (-self.ball_params.get_current_position()[1] + defense_point_y)**2) != 0:
                t = self.r_offset / np.sqrt((self.ball_params.get_current_position()[0] - defense_point_x)**2 + (-self.ball_params.get_current_position()[1] + defense_point_y)**2)
            else:
                return 0, 0, 0

            keeper_position_x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1) + (self.ball_params.get_current_position()[0] - defense_point_x) * t
            keeper_position_y = -((a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2) + (-self.ball_params.get_current_position()[1] + defense_point_y) * t)



        else:
            a1 = self.ball_params.get_current_position()[1] - defense_point_y
            b1 = -defense_point_x + self.ball_params.get_current_position()[0]
            c1 = -self.ball_params.get_current_position()[0] * -a1 + self.ball_params.get_current_position()[1] * -b1

            a2 = b1
            b2 = -a1

            if self.ball_params.get_current_position()[1] > 0.:
                c2 =self.goal_right[0] * -a2 - self.goal_right[1] * -b2
            else:
                c2 =self.goal_left[0] * -a2 - self.goal_left[1] * -b2

            if np.sqrt((-self.ball_params.get_current_position()[0] + defense_point_x)**2 + (self.ball_params.get_current_position()[1] - defense_point_y)**2) != 0:
                t = self.r_offset / np.sqrt((-self.ball_params.get_current_position()[0] + defense_point_x)**2 + (self.ball_params.get_current_position()[1] - defense_point_y)**2)
            else:
                return 0, 0, 0

            keeper_position_x = -((b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1) + (-self.ball_params.get_current_position()[0] + defense_point_x) * t)
            keeper_position_y = (a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2) + (self.ball_params.get_current_position()[1] - defense_point_y) * t

        keeper_position_theta = np.arctan2((self.ball_params.get_current_position()[1] - self.ctrld_robot.get_current_position()[1]) , (self.ball_params.get_current_position()[0] - self.ctrld_robot.get_current_position()[0]))

        return keeper_position_x, keeper_position_y, keeper_position_theta

    def calc_line(self, team_side, y):
        if team_side == "right":
            left_line = ((self.ball_params.get_current_position()[0] - self.goal_left[0]) / (-self.ball_params.get_current_position()[1] + self.goal_left[1]) * (-y + self.goal_left[1]) + self.goal_left[0])
            right_line = ((self.ball_params.get_current_position()[0] - self.goal_right[0]) / (-self.ball_params.get_current_position()[1] + self.goal_right[1]) * (-y + self.goal_right[1]) + self.goal_right[0])

        if team_side == "left":
            left_line = ((self.ball_params.get_current_position()[0] + self.goal_left[0]) / (-self.ball_params.get_current_position()[1] - self.goal_left[1]) * (-y - self.goal_left[1]) - self.goal_left[0])
            right_line = ((self.ball_params.get_current_position()[0] + self.goal_right[0]) / (-self.ball_params.get_current_position()[1] - self.goal_right[1]) * (-y - self.goal_right[1]) - self.goal_right[0])

        return right_line, left_line

    def detect_obstacle(self):
        friend_obstacle = []
        enemy_obstacle = []

        if self.team_side == "right":
            if self.goal_right[1] < self.ball_params.get_current_position()[1] < self.goal_left[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] > right_line \
                            and self.friend[i].get_current_position()[0] > left_line:
                            friend_obstacle.append(i)

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] > right_line \
                        and self.enemy[i].get_current_position()[0] > left_line:
                        enemy_obstacle.append(i)

            if self.ball_params.get_current_position()[1] < self.goal_right[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] < right_line \
                            and self.friend[i].get_current_position()[0] > left_line:
                            friend_obstacle.append(i)

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] < right_line \
                        and self.enemy[i].get_current_position()[0] > left_line:
                        enemy_obstacle.append(i)

            if self.goal_left[1] < self.ball_params.get_current_position()[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] > right_line \
                            and self.friend[i].get_current_position()[0] < left_line:
                            friend_obstacle.append(i)

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] > right_line \
                        and self.enemy[i].get_current_position()[0] < left_line:
                        enemy_obstacle.append(i)


        else:
            if self.goal_right[1] < self.ball_params.get_current_position()[1] < self.goal_left[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] < right_line \
                            and self.friend[i].get_current_position()[0] < left_line:
                            friend_obstacle.append(i)

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] < right_line \
                        and self.enemy[i].get_current_position()[0] < left_line:
                        enemy_obstacle.append(i)

            if self.ball_params.get_current_position()[1] < self.goal_right[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] < right_line \
                            and self.friend[i].get_current_position()[0] > left_line:
                            friend_obstacle.append(i)

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] < right_line \
                        and self.enemy[i].get_current_position()[0] > left_line:
                        enemy_obstacle.append(i)

            if self.goal_left[1] < self.ball_params.get_current_position()[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] > right_line \
                            and self.friend[i].get_current_position()[0] < left_line:
                            friend_obstacle.append(i)

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] > right_line \
                        and self.enemy[i].get_current_position()[0] < left_line:
                        enemy_obstacle.append(i)

        return friend_obstacle, enemy_obstacle

    def culc_sheltered_area(self, obstacle):
        if self.team_side == "right":
            ball_x_obst_coord = self.ball_params.get_current_position()[0] - obstacle[0]
            ball_y_obst_coord = -self.ball_params.get_current_position()[1] + obstacle[1]

            a = 1. + (ball_x_obst_coord**2. / ball_y_obst_coord**2.)
            b = -(2. * self.ctrld_robot.size_r**2. * ball_x_obst_coord / ball_y_obst_coord**2)
            c = (self.ctrld_robot.size_r**4. / ball_y_obst_coord**2.) - self.ctrld_robot.size_r**2

            if (b**2. - 4. * a * c <= 0.) or (2. * a == 0.):
                return 0, 0
            obst_contact_x1 = (-b - np.sqrt(b**2. - 4. * a * c)) / (2. * a)
            obst_contact_y1 = 1. / ball_y_obst_coord * (-ball_x_obst_coord * obst_contact_x1 + self.ctrld_robot.size_r**2.)
            obst_contact_x2 = (-b + np.sqrt(b**2. - 4. * a * c)) / (2. * a)
            obst_contact_y2 = 1. / ball_y_obst_coord * (-ball_x_obst_coord * obst_contact_x2 + self.ctrld_robot.size_r**2.)

            goal_center_x = self.goal_left[0]

            y_goal_l1 = 1. / obst_contact_y1 * (-obst_contact_x1 * goal_center_x + obst_contact_x1 * obstacle[0] + obst_contact_y1 * -obstacle[1] + self.ctrld_robot.size_r**2.)
            y_goal_l2 = 1. / obst_contact_y2 * (-obst_contact_x2 * goal_center_x + obst_contact_x2 * obstacle[0] + obst_contact_y2 * -obstacle[1] + self.ctrld_robot.size_r**2.)

            if -y_goal_l1 <= -y_goal_l2:
                shel_r = -y_goal_l1
                shel_l = -y_goal_l2
            else:
                shel_r = -y_goal_l2
                shel_l = -y_goal_l1

        else:
            ball_x_obst_coord = -self.ball_params.get_current_position()[0] + obstacle[0]
            ball_y_obst_coord = self.ball_params.get_current_position()[1] - obstacle[1]

            a = 1. + (ball_x_obst_coord**2. / ball_y_obst_coord**2.)
            b = -(2. * self.ctrld_robot.size_r**2. * ball_x_obst_coord / ball_y_obst_coord**2)
            c = (self.ctrld_robot.size_r**4. / ball_y_obst_coord**2.) - self.ctrld_robot.size_r**2

            if (b**2. - 4. * a * c <= 0.) or (2. * a == 0.):
                return 0, 0
            obst_contact_x1 = (-b - np.sqrt(b**2. - 4. * a * c)) / (2. * a)
            obst_contact_y1 = 1. / ball_y_obst_coord * (-ball_x_obst_coord * obst_contact_x1 + self.ctrld_robot.size_r**2.)
            obst_contact_x2 = (-b + np.sqrt(b**2. - 4. * a * c)) / (2. * a)
            obst_contact_y2 = 1. / ball_y_obst_coord * (-ball_x_obst_coord * obst_contact_x2 + self.ctrld_robot.size_r**2.)

            goal_center_x = self.goal_left[0]

            y_goal_l1 = 1. / obst_contact_y1 * (-obst_contact_x1 * goal_center_x + obst_contact_x1 * -obstacle[0] + obst_contact_y1 * obstacle[1] + self.ctrld_robot.size_r**2.)
            y_goal_l2 = 1. / obst_contact_y2 * (-obst_contact_x2 * goal_center_x + obst_contact_x2 * -obstacle[0] + obst_contact_y2 * obstacle[1] + self.ctrld_robot.size_r**2.)

            if y_goal_l1 <= y_goal_l2:
                shel_r = y_goal_l1
                shel_l = y_goal_l2
            else:
                shel_r = y_goal_l2
                shel_l = y_goal_l1

        return shel_r, shel_l

    def sort_sheltered_area(self, shel_r, shel_l):
        shel_area = dict(zip(shel_r, shel_l))
        shel_area = sorted(shel_area.items(), key=lambda x:x[0])
        print shel_area

    def culc_defense_point(self, shel_r, shel_l):
        print shel_r, shel_l


    def keeper(self):
        shel_r = []
        shel_l = []
        friend_obstacle, enemy_obstacle = self.detect_obstacle()
        #print friend_obstacle
        if friend_obstacle != [] or enemy_obstacle != []:
            for i in friend_obstacle:
                shel_r.append(self.culc_sheltered_area(self.friend[i].get_current_position())[0])
                shel_l.append(self.culc_sheltered_area(self.friend[i].get_current_position())[1])
            for i in enemy_obstacle:
                shel_r.append(self.culc_sheltered_area(self.enemy[i].get_current_position())[0])
                shel_l.append(self.culc_sheltered_area(self.enemy[i].get_current_position())[1])
            self.sort_sheltered_area(shel_r, shel_l)
        x, y, theta = self.calc_keeper_position(-6., 0.)
        self.pid.pid_linear(x, y, theta)


