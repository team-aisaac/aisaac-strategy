import math


class RobotDefence:
    def __init__(self, status, kick):
        # type: (robot_status.RobotStatus) -> None
        self.status = status
        self.ball_params = status.pid.ball_params
        self.pid = status.pid

        self.def1_pos_x = 0.0
        self.def1_pos_y = 0.0
        self.def2_pos_x = 0.0
        self.def2_pos_y = 0.0

        self.kick = kick

    def def_pos_callback(self, msg):
        self.def1_pos_x = msg.def1_pos_x
        self.def1_pos_y = msg.def1_pos_y
        self.def2_pos_x = msg.def2_pos_x
        self.def2_pos_y = msg.def2_pos_y

    def move_defence(self, x, y):
        pose_theta = math.atan2( (self.ball_params.get_current_position()[1] - y) , (self.ball_params.get_current_position()[0] - x) )
        self.pid.pid_linear(x, y, pose_theta)
        self.kick.receive_and_clear((x, y))

    def defence2(self):
        pose_theta = math.atan2( (self.ball_params.get_current_position()[1] - self.def2_pos_y) , (self.ball_params.get_current_position()[0] - self.def2_pos_x) )
        self.pid.pid_linear(self.def2_pos_x, self.def2_pos_y, pose_theta)
        self.kick.receive_and_clear((self.def2_pos_x, self.def2_pos_y))
