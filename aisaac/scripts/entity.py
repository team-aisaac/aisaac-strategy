import numpy as np
import random

class Entity():
    def __init__(self):
        self.field_x_size = 12000.
        self.field_y_size = 9000.
        self.size_r = 0.
        self.current_position_x = 0.
        self.current_position_y = 0.
        #self.current_position_x = random.uniform(-self.field_x_size/2, self.field_x_size/2)
        #self.current_position_y = random.uniform(-self.field_y_size/2, self.field_y_size/2)
        self.current_orientation = 0.


    def set_current_position(self, x, y, theta):
        self.current_position_x = x
        self.current_position_y = y
        self.current_orientation = theta

    def show_current_parameters(self):
        return "size_r:{}, current_position_x:{}, current_position_y:{}, current_orientation:{}".format(
                self.size_r, self.current_position_x, self.current_position_y, self.current_orientation)

    def get_current_position(self):
        return self.current_position_x, self.current_position_y, self.current_orientation

    def get_current_parameter_xy(self):
        return self.current_position_x, self.current_position_y

class Robot(Entity):
    def __init__(self):
        ###super().__init__()
        self.size_r = 90.
        self.front_degree = 11.86 * 2
        self.max_velocity = 2000. #mm/s
        self.future_position_x = 0.
        self.future_position_y = 0.
        self.future_orientation = 0.
        self.current_position_x = 0.
        self.current_position_y = 0.
        self.current_orientation = 0.
        self.has_a_ball = False
        self.position = ""
        self.velocity_surge = 0
        self.velocity_sway = 0
        self.omega = 0

    def set_future_position(self, x, y, theta):
        self.future_position_x = x
        self.future_position_y = y
        self.future_orientation = theta

    def set_current_position(self, x, y, theta):
        None
    def set_current_velosity(self, x, y, theta):
        None

    def get_future_position(self):
        return self.future_position_x, self.future_position_y, self.future_orientation

    def get_future_parameter_xy(self):
        return self.future_position_x, self.future_position_y

class Ball(Entity):
    def __init__(self):
        ###super().__init__()
        self.size_r = 21.5
        self.current_position_x = 0.
        self.current_position_y = 0.




if __name__ == "__main__":
    robot_0 = robot()
    print(robot_0.show_parameters())
    ball_ = ball()
    print(ball_.show_parameters())
