#!/usr/bin/env  python
# coding:utf-8
import numpy as np
import rospy
from common import functions


class RobotKeeper(object):
    def __init__(self, kick):
        # type: (robot_kick.RobotKick) -> None

        self.robot_id = kick.pid.robot_id
        self.ctrld_robot = kick.pid.ctrld_robot
        self.friend = kick.pid.friend
        self.enemy = kick.pid.enemy
        self.ball_params = kick.ball_params
        self.kick = kick
        self.team_side = "left" # CONSAIが変換してくれることが発覚したため常にleft
        self.goal_right = [6, -0.620]
        self.goal_left = [6, 0.620]
        self.r_offset = self.ctrld_robot.size_r
        self.objects = kick.pid.objects
        self._ball_in_friend_penalty_start_time = rospy.Time.now()


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
                for i in self.objects.get_active_robot_ids():
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.objects.get_robot_by_id(i).get_current_position()[1])
                        if self.objects.get_robot_by_id(i).get_current_position()[0] > right_line \
                            and self.objects.get_robot_by_id(i).get_current_position()[0] > left_line:
                            friend_obstacle.append(i)

                for i in self.objects.get_active_enemy_ids():
                    right_line, left_line = self.calc_line(self.team_side, self.objects.get_enemy_by_id(i).get_current_position()[1])
                    if self.objects.get_enemy_by_id(i).get_current_position()[0] > right_line \
                        and self.objects.get_enemy_by_id(i).get_current_position()[0] > left_line:
                        enemy_obstacle.append(i)

            if self.ball_params.get_current_position()[1] < self.goal_right[1]:
                for i in self.objects.get_active_robot_ids():
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.objects.get_robot_by_id(i).get_current_position()[1])
                        if self.objects.get_robot_by_id(i).get_current_position()[0] < right_line \
                            and self.objects.get_robot_by_id(i).get_current_position()[0] > left_line:
                            friend_obstacle.append(i)

                for i in self.objects.get_active_enemy_ids():
                    right_line, left_line = self.calc_line(self.team_side, self.objects.get_enemy_by_id(i).get_current_position()[1])
                    if self.objects.get_enemy_by_id(i).get_current_position()[0] < right_line \
                        and self.objects.get_enemy_by_id(i).get_current_position()[0] > left_line:
                        enemy_obstacle.append(i)

            if self.goal_left[1] < self.ball_params.get_current_position()[1]:
                for i in self.objects.get_active_robot_ids():
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.objects.get_robot_by_id(i).get_current_position()[1])
                        if self.objects.get_robot_by_id(i).get_current_position()[0] > right_line \
                            and self.objects.get_robot_by_id(i).get_current_position()[0] < left_line:
                            friend_obstacle.append(i)

                for i in self.objects.get_active_enemy_ids():
                    right_line, left_line = self.calc_line(self.team_side, self.objects.get_enemy_by_id(i).get_current_position()[1])
                    if self.objects.get_enemy_by_id(i).get_current_position()[0] > right_line \
                        and self.objects.get_enemy_by_id(i).get_current_position()[0] < left_line:
                        enemy_obstacle.append(i)


        else:
            if self.goal_right[1] < self.ball_params.get_current_position()[1] < self.goal_left[1]:
                for i in self.objects.get_active_robot_ids():
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.objects.get_robot_by_id(i).get_current_position()[1])
                        if self.objects.get_robot_by_id(i).get_current_position()[0] < right_line \
                            and self.objects.get_robot_by_id(i).get_current_position()[0] < left_line:
                            friend_obstacle.append(i)

                for i in self.objects.get_active_enemy_ids():
                    right_line, left_line = self.calc_line(self.team_side, self.objects.get_enemy_by_id(i).get_current_position()[1])
                    if self.objects.get_enemy_by_id(i).get_current_position()[0] < right_line \
                        and self.objects.get_enemy_by_id(i).get_current_position()[0] < left_line:
                        enemy_obstacle.append(i)

            if self.ball_params.get_current_position()[1] < self.goal_right[1]:
                for i in self.objects.get_active_robot_ids():
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.objects.get_robot_by_id(i).get_current_position()[1])
                        if self.objects.get_robot_by_id(i).get_current_position()[0] < right_line \
                            and self.objects.get_robot_by_id(i).get_current_position()[0] > left_line:
                            friend_obstacle.append(i)

                for i in self.objects.get_active_enemy_ids():
                    right_line, left_line = self.calc_line(self.team_side, self.objects.get_enemy_by_id(i).get_current_position()[1])
                    if self.objects.get_enemy_by_id(i).get_current_position()[0] < right_line \
                        and self.objects.get_enemy_by_id(i).get_current_position()[0] > left_line:
                        enemy_obstacle.append(i)

            if self.goal_left[1] < self.ball_params.get_current_position()[1]:
                for i in self.objects.get_active_robot_ids():
                    if i != self.robot_id:
                        right_line, left_line = self.calc_line(self.team_side, self.objects.get_robot_by_id(i).get_current_position()[1])
                        if self.objects.get_robot_by_id(i).get_current_position()[0] > right_line \
                            and self.objects.get_robot_by_id(i).get_current_position()[0] < left_line:
                            friend_obstacle.append(i)

                for i in self.objects.get_active_enemy_ids():
                    right_line, left_line = self.calc_line(self.team_side, self.objects.get_enemy_by_id(i).get_current_position()[1])
                    if self.objects.get_enemy_by_id(i).get_current_position()[0] > right_line \
                        and self.objects.get_enemy_by_id(i).get_current_position()[0] < left_line:
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
        if self.team_side == "right":
            shel_area = sorted(dict(zip(shel_l, shel_r)).items(), key=lambda x:-x[0])
        else:
            shel_area = sorted(dict(zip(shel_l, shel_r)).items(), key=lambda x:x[0])

        sort_area = []
        while(len(shel_area) != 0):
            comb_shel, shel_area = self.combine_sheltered_area(shel_area, shel_area[0])
            sort_area.append(comb_shel)

        return sort_area

    def combine_sheltered_area(self, shel_area, shel):
        if self.team_side == "right":
            comb_shel = [shel[0], shel[1]]
            for i in reversed(range(len(shel_area))):
                if shel[1] <= shel_area[i][1]:
                    del shel_area[i]

            shel_r_min = shel[1]
            for i in range(len(shel_area)):
                if shel[0] > shel_area[i][0] > shel[1]:
                    comb_shel[0] = shel[0]
                    if shel_area[i][1] < shel_r_min:
                        shel_r_min = shel_area[i][1]
            if shel_r_min != shel[1]:
                comb_shel[1] = shel_r_min
                comb_shel, shel_area = self.combine_sheltered_area(shel_area, comb_shel)
        else:
            comb_shel = [shel[0], shel[1]]
            for i in reversed(range(len(shel_area))):
                if shel[1] >= shel_area[i][1]:
                    del shel_area[i]

            shel_r_min = shel[1]
            for i in range(len(shel_area)):
                if shel[0] < shel_area[i][0] < shel[1]:
                    comb_shel[0] = shel[0]
                    if shel_area[i][1] > shel_r_min:
                        shel_r_min = shel_area[i][1]
            if shel_r_min != shel[1]:
                comb_shel[1] = shel_r_min
                comb_shel, shel_area = self.combine_sheltered_area(shel_area, comb_shel)

        return comb_shel, shel_area



    def culc_defense_point(self, shel):
        defense_area_r = []
        defense_area_l = []
        defense_point_x = 0
        defense_point_y = 0

        if self.team_side == "right":
            if shel == [[]]:
                return self.goal_left[0], 0
            defense_area_array = np.append(np.ravel(shel), [self.goal_left[1], self.goal_right[1]])
            defense_area_array.sort()
            defense_area_array = defense_area_array[::-1]
            defense_area = []
            for i in range(len(defense_area_array) - 1):
                for j in range(len(shel)):
                    if (not(shel[j][0] >= defense_area_array[i] >= shel[j][1]) \
                        or not(shel[j][0] >= defense_area_array[i + 1] >= shel[j][1])) \
                        and (self.goal_left[1] >= defense_area_array[i] >= self.goal_right[1]) \
                        and (self.goal_left[1] >= defense_area_array[i + 1] >= self.goal_right[1]):
                        defense_area.append([defense_area_array[i], defense_area_array[i+1]])

            defense_area_max = 0
            for i in range(len(defense_area)):
                if defense_area[i][0] - defense_area[i][1] > defense_area_max:
                    defense_area_max = defense_area[i][0] - defense_area[i][1]
                    defense_point_y = (defense_area[i][0] + defense_area[i][1]) / 2.
            defense_point_x = self.goal_left[0]

        else:
            if shel == [[]]:
                return -self.goal_left[0], 0
            defense_area_array = np.append(np.ravel(shel), [-self.goal_left[1], -self.goal_right[1]])
            defense_area_array.sort()
            defense_area_array = defense_area_array[::-1]
            defense_area = []
            for i in range(len(defense_area_array) - 1):
                for j in range(len(shel)):
                    if (not(shel[j][0] >= defense_area_array[i] >= shel[j][1]) \
                        or not(shel[j][0] >= defense_area_array[i + 1] >= shel[j][1])) \
                        and (self.goal_left[1] >= defense_area_array[i] >= self.goal_right[1]) \
                        and (self.goal_left[1] >= defense_area_array[i + 1] >= self.goal_right[1]):
                        defense_area.append([defense_area_array[i], defense_area_array[i+1]])

            defense_area_max = 0
            for i in range(len(defense_area)):
                if defense_area[i][0] - defense_area[i][1] > defense_area_max:
                    defense_area_max = defense_area[i][0] - defense_area[i][1]
                    defense_point_y = (defense_area[i][0] + defense_area[i][1]) / 2.
            defense_point_x = -self.goal_left[0]

        return defense_point_x, defense_point_y




    def keeper(self):
        if functions.in_penalty_area(self.ball_params.get_current_position()) == "friend":
            if (rospy.Time.now() - self._ball_in_friend_penalty_start_time).to_sec() > 3.0:
                self.kick.pass_ball(self.ctrld_robot.get_pass_target_position()[0],
                                    self.ctrld_robot.get_pass_target_position()[1],
                                    is_tip_kick=True,
                                    ignore_penalty_area=True)
                return
        else:
            self._ball_in_friend_penalty_start_time = rospy.Time.now()

        shel_r = []
        shel_l = []
        if self.team_side == "right":
            defense_point_x = self.goal_left[0]
        else:
            defense_point_x = -self.goal_left[0]
        defense_point_y = 0

        #壁検知
        friend_obstacle, enemy_obstacle = self.detect_obstacle()

        #壁による守備不要域取得
        if friend_obstacle != [] or enemy_obstacle != []:
            for i in friend_obstacle:
                shel_r.append(self.culc_sheltered_area(self.objects.get_robot_by_id(i).get_current_position())[0])
                shel_l.append(self.culc_sheltered_area(self.objects.get_robot_by_id(i).get_current_position())[1])
            """
            for i in enemy_obstacle:
                shel_r.append(self.culc_sheltered_area(self.enemy[i].get_current_position())[0])
                shel_l.append(self.culc_sheltered_area(self.enemy[i].get_current_position())[1])
            """
            shel = self.sort_sheltered_area(shel_r, shel_l)

            #キーパー守備位置
            defense_point_x, defense_point_y = self.culc_defense_point(shel)
        x, y, theta = self.calc_keeper_position(defense_point_x, defense_point_y)
        self.kick.receive_ball_keeper((x, y))

