# !/usr/bin/env  python
# coding:utf-8

from world.objects import Objects


class WorldState(object):
    def __init__(self, objects):
        # type: (Objects) -> None
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

