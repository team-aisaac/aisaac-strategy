import numpy as np
import random

class Entity(object):
    def __init__(self):
        self.field_x_size = 12000.
        self.field_y_size = 9000.
        self.size_r = 0.
        self._current_position_x = 0.
        self._current_position_y = 0.
        #self._current_position_x = random.uniform(-self.field_x_size/2, self.field_x_size/2)
        #self._current_position_y = random.uniform(-self.field_y_size/2, self.field_y_size/2)
        self._current_orientation = 0.

        self._future_position_x = 0.
        self._future_position_y = 0.
        self._future_orientation = 0.


    def set_current_position(self, x, y, theta=None):
        self._current_position_x = x
        self._current_position_y = y

        if theta:
            self._current_orientation = theta

    def get_current_position(self, theta=None):
        if theta:
            return self._current_position_x, self._current_position_y, self._current_orientation
        else:
            return self._current_position_x, self._current_position_y

    def get_current_orientation(self):
        return self._current_orientation

    def set_current_velocity(self, vx, vy, vtheta=None):
        self._current_velocity_x = vx
        self._current_velocity_y = vy

        if vtheta:
            self._current_velocity_orientation = vtheta

    def set_current_velocity_orientation(self, vtheta):
        self._current_velocity_orientation = vtheta

    def get_current_velocity(self):
        return self._current_velocity_x, self._current_velocity_y

    def get_current_velocity_orientation(self):
        return self._current_velocity_orientation

    def set_future_position(self, x, y, theta=None):
        self._future_position_x = x
        self._future_position_y = y

        if theta:
            self._future_orientation = theta

    def set_future_orientation(self, theta):
        self._future_orientation = theta

    def get_future_position(self, theta=None):
        if theta:
            return self._future_position_x, self._future_position_y, self._future_orientation
        else:
            return self._future_position_x, self._future_position_y

    def get_future_orientation(self):
        return self._future_orientation



class Robot(Entity):
    def __init__(self):
        super(Robot, self).__init__()
        self.size_r = 90.0 / 1000

        self.front_degree = 11.86 * 2
        self.max_velocity = 2000. #mm/s

        self._has_a_ball = False

        self._pass_target_pos_x = 0.
        self._pass_target_pos_y = 0.

        self.position = ""
        self.velocity_surge = 0
        self.velocity_sway = 0
        self.omega = 0

    def has_a_ball(self):
        return self._has_a_ball

    def set_pass_target_position(self, x, y):
        self._pass_target_pos_x = x
        self._pass_target_pos_y = y

    # def set_current_position(self, x, y, theta):
    #     None
    # def set_current_velosity(self, x, y, theta):
    #     None

    # def get_future_position(self):
    #     return self.future_position_x, self.future_position_y, self.future_orientation

    def get_pass_target_position(self):
        return self._pass_target_pos_x, self._pass_target_pos_y

    # def get_future_parameter_xy(self):
    #     return self._future_position_x, self._future_position_y


class Ball(Entity):
    def __init__(self):
        super(Ball, self).__init__()
        self.size_r = 21.5 / 1000

        self._line_a = 0
        self._line_b = 0

    def get_line_a(self):
        return self._line_a

    def get_line_b(self):
        return self._line_b

    def set_line_a(self, a):
        self._line_a = a

    def set_line_b(self, b):
        self._line_b = b

