import math

class RobotDefence:
    def __init__(self, ball_params, pid, cmd, status, command_pub):
        self.ball_params = ball_params
        self.pid = pid
        self.status = status
        self.cmd = cmd
        self.command_pub = command_pub

        self.def1_pos_x = 0.0
        self.def1_pos_y = 0.0
        self.def2_pos_x = 0.0
        self.def2_pos_y = 0.0

    def def_pos_callback(self, msg):
        self.def1_pos_x = msg.def1_pos_x
        self.def1_pos_y = msg.def1_pos_y
        self.def2_pos_x = msg.def2_pos_x
        self.def2_pos_y = msg.def2_pos_y

    def defence1(self):
        pose_theta = math.atan2( (self.ball_params.get_current_position()[1] - self.def1_pos_y) , (self.ball_params.get_current_position()[0] - self.def1_pos_x) )
        self.pid.pid_linear(self.def1_pos_x, self.def1_pos_y, pose_theta)

    def defence2(self):
        pose_theta = math.atan2( (self.ball_params.get_current_position()[1] - self.def2_pos_y) , (self.ball_params.get_current_position()[0] - self.def2_pos_x) )
        self.pid.pid_linear(self.def2_pos_x, self.def2_pos_y, pose_theta)
