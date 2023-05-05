#!/usr/bin/env  python
# coding:utf-8

class RobotStatus:
    def __init__(self, pid, cmd_v2):
        # type: (robot_pid.RobotPid, cmd_v2) -> None
        self.robot_status = "none"

        self.pid = pid
        self.ctrld_robot = pid.ctrld_robot
        self.cmd_v2 = cmd_v2

        self.ctrld_robot.set_future_position(0., 0., 0.)

    def status_callback(self, msg):
        if msg.status == "move_linear" and (self.ctrld_robot.get_future_position()[0] != msg.pid_goal_pos_x or self.ctrld_robot.get_future_position()[1] != msg.pid_goal_pos_y or self.ctrld_robot.get_future_orientation() != msg.pid_goal_theta):
            self.pid.goal_pos_init_flag = True

        self.robot_status = msg.status
        self.ctrld_robot.set_future_position(
            msg.pid_goal_pos_x,
            msg.pid_goal_pos_y,
            msg.pid_goal_theta)
        
        self.cmd_v2.goal_pose.x = msg.pid_goal_pos_x
        self.cmd_v2.goal_pose.y = msg.pid_goal_pos_y
        self.cmd_v2.goal_pose.theta = msg.pid_goal_theta

        self.pid.pid_circle_center_x = msg.pid_circle_center_x
        self.pid.pid_circle_center_y = msg.pid_circle_center_y
        self.ctrld_robot.set_pass_target_position(msg.pass_target_pos_x, msg.pass_target_pos_y)
        self.cmd_v2.ball_goal.x = msg.pass_target_pos_x
        self.cmd_v2.ball_goal.y = msg.pass_target_pos_y


