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
        left_line = ((self.ball_params.get_current_position()[0] - self.goal_left[0]) / (-self.ball_params.get_current_position()[1] + self.goal_left[1]) * (-y + self.goal_left[1]) + self.goal_left[0])
        right_line = ((self.ball_params.get_current_position()[0] - self.goal_right[0]) / (-self.ball_params.get_current_position()[1] + self.goal_right[1]) * (-y + self.goal_right[1]) + self.goal_right[0])

        return right_line, left_line

    def detect_obstacle(self):
        if self.team_side == "right":
            if self.goal_right[1] < self.ball_params.get_current_position()[1] < self.goal_left[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] > right_line \
                            and self.friend[i].get_current_position()[0] > left_line:
                            print "AAA"

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] > right_line \
                        and self.enemy[i].get_current_position()[0] > left_line:
                        print "BBB"

            if self.ball_params.get_current_position()[1] < self.goal_right[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] < right_line \
                            and self.friend[i].get_current_position()[0] > left_line:
                            print "AAA"

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] < right_line \
                        and self.enemy[i].get_current_position()[0] > left_line:
                        print "BBB"

            if self.goal_left[1] < self.ball_params.get_current_position()[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] > right_line \
                            and self.friend[i].get_current_position()[0] < left_line:
                            print "AAA"

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] > right_line \
                        and self.enemy[i].get_current_position()[0] < left_line:
                        print "BBB"

        """
        else:
            if self.goal_right[1] < self.ball_params.get_current_position()[1] < self.goal_left[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] < right_line \
                            and self.friend[i].get_current_position()[0] < left_line:
                            print "AAA"

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] < right_line \
                        and self.enemy[i].get_current_position()[0] < left_line:
                        print "BBB"

            if self.ball_params.get_current_position()[1] < self.goal_right[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] < right_line \
                            and self.friend[i].get_current_position()[0] > left_line:
                            print "AAA"

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] < right_line \
                        and self.enemy[i].get_current_position()[0] > left_line:
                        print "BBB"

            if self.goal_left[1] < self.ball_params.get_current_position()[1]:
                for i in range(len(self.friend)):
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.friend[i].get_current_position()[1])
                        if self.friend[i].get_current_position()[0] > right_line \
                            and self.friend[i].get_current_position()[0] < left_line:
                            print "AAA"

                for i in range(len(self.enemy)):
                    right_line, left_line = self.calc_line(self.team_side, self.enemy[i].get_current_position()[1])
                    if self.enemy[i].get_current_position()[0] > right_line \
                        and self.enemy[i].get_current_position()[0] < left_line:
                        print "BBB"
        """
    def keeper(self):
        self.detect_obstacle()
        x, y, theta = self.calc_keeper_position(-6., 0.)
        self.pid.pid_linear(x, y, theta)


