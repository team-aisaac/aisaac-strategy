#!/usr/bin/env  python
# coding:utf-8
import numpy as np

class RobotKeeper:
    def __init__(self, ball_params, ctrld_robot, pid, cmd, status, command_pub):
        self.ball_params = ball_params
        self.ctrld_robot = ctrld_robot
        self.pid = pid
        self.status = status
        self.cmd = cmd
        self.command_pub = command_pub

        self.goal_right = [6, -0.620]
        self.goal_left = [6, 0.620]
        self.r_offset = self.ctrld_robot.size_r


    def calc_keeper_position(self, defense_point_x, defense_point_y):
        if defense_point_x > 0:
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


    def keeper(self):
        x, y, theta = self.calc_keeper_position(-6., 0.)
        self.pid.pid_linear(x, y, theta)


