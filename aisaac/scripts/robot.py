#!/usr/bin/env  python
# coding:utf-8
import rospy

from consai_msgs.msg import robot_commands
from aisaac.msg import Status, Ball_sub_params, Def_pos
from aisaac.srv import pid

from robot_kick import RobotKick
from robot_pid import RobotPid
from robot_status import RobotStatus
from robot_defence import RobotDefence
from robot_keeper import RobotKeeper
from context import RobotContext

from objects import Objects
from robot_command_publisher_wrapper import RobotCommandPublisherWrapper

from filter import kalman_filter, identity_filter

import config
import numpy as np
import functions

ROBOT_LOOP_RATE = config.ROBOT_LOOP_RATE


class Robot(object):
    def __init__(self, robot_id=None):
        rospy.init_node("robot")
        self.robot_color = str(rospy.get_param("~robot_color"))

        if not robot_id:
            self.robot_id = str(rospy.get_param("~robot_num"))
        else:
            self.robot_id = robot_id

        self.robot_total = config.NUM_FRIEND_ROBOT
        self.enemy_total = config.NUM_ENEMY_ROBOT

        self.cmd = robot_commands() # type: robot_commands
        self.cmd.kick_speed_x = 0
        self.cmd.kick_speed_z = 0
        self.cmd.dribble_power = 0

        self._command_pub = RobotCommandPublisherWrapper(self.robot_color, self.robot_id)

        # Composition
        self.objects = Objects(self.robot_color, self.robot_total, self.enemy_total)
        self.ctrld_robot = self.objects.robot[int(self.robot_id)] # type: entity.Robot

        self.robot_friend = self.objects.robot
        self.robot_enemy = self.objects.enemy

        self.ball_params = self.objects.ball

        self.pid = RobotPid(self.robot_id, self.objects, self.cmd)
        self.status = RobotStatus(self.pid)
        self.kick = RobotKick(self.pid, self.cmd, self.status)
        self.defence = RobotDefence(self.status, self.kick)
        self.keeper = RobotKeeper(self.kick)

        # listner 起動
        # self.odom_listener()
        # self.goal_pose_listener()
        # self.target_pose_listener()
        self.status_listener()
        self.set_pid_server()
        self.def_pos_listener()
        rospy.Timer(rospy.Duration(1.0/30.0), self.pid.replan_timer_callback)

    def store_and_publish_commands(self):

        self.ctrld_robot.update_expected_velocity_context(self.cmd.vel_x,
                                                          self.cmd.vel_y,
                                                          self.cmd.omega)

        self.cmd.vel_surge, self.cmd.vel_sway \
            = functions.clip_vector2((self.cmd.vel_surge, self.cmd.vel_sway), 0.075)

        self.ctrld_robot.handle_loop_callback()

        self._command_pub.publish(self.cmd)
        # self.reset_cmd()

    def reset_cmd(self):
        default_cmd = robot_commands()
        default_cmd.kick_speed_x = 0
        default_cmd.kick_speed_z = 0
        default_cmd.dribble_power = 0

        self.cmd.vel_x = default_cmd.vel_x
        self.cmd.vel_y = default_cmd.vel_y
        self.cmd.vel_surge = default_cmd.vel_surge
        self.cmd.vel_sway = default_cmd.vel_sway
        self.cmd.omega = default_cmd.omega
        self.cmd.theta = default_cmd.theta
        self.cmd.kick_speed_x = default_cmd.kick_speed_x
        self.cmd.kick_speed_z = default_cmd.kick_speed_z
        self.cmd.dribble_power = default_cmd.dribble_power


    def shutdown_cmd(self):
        self.reset_cmd()
        self.cmd.shutdown_flag = True
        self.store_and_publish_commands()


    def run(self):
        # Loop 処理
        loop_rate = rospy.Rate(ROBOT_LOOP_RATE)
        rospy.loginfo("start robot node: "+self.robot_id)

        while not rospy.is_shutdown():
            # start = time.time()

            # カルマンフィルタ,恒等関数フィルタの適用
            # vision_positionからcurrent_positionを決定してつめる
            for robot in self.robot_friend:
                if robot.get_id() == self.ctrld_robot.get_id():
                    kalman_filter(self.ctrld_robot)
                    #identity_filter(self.ctrld_robot)
                else:
                    identity_filter(robot)
            for enemy in self.robot_enemy:
                identity_filter(enemy)


            if self.status.robot_status == "move_linear":
                self.pid.pid_linear(self.ctrld_robot.get_future_position()[0],
                                    self.ctrld_robot.get_future_position()[1],
                                    self.ctrld_robot.get_future_orientation())
            elif self.status.robot_status == "move_circle":
                self.pid.pid_circle(self.pid.pid_circle_center_x, self.pid.pid_circle_center_y,
                                    self.ctrld_robot.get_future_position()[0],
                                    self.ctrld_robot.get_future_position()[1],
                                    self.ctrld_robot.get_future_orientation())

            elif self.status.robot_status == "pass":
                self.kick.pass_ball(self.ctrld_robot.get_pass_target_position()[0],
                                    self.ctrld_robot.get_pass_target_position()[1])
            elif self.status.robot_status == "pass_chip":
                self.kick.pass_ball(self.ctrld_robot.get_pass_target_position()[0],
                                    self.ctrld_robot.get_pass_target_position()[1],
                                    is_tip_kick=True)
            elif self.status.robot_status == "prepare_pass":
                self.kick.pass_ball(self.ctrld_robot.get_pass_target_position()[0],
                                    self.ctrld_robot.get_pass_target_position()[1],
                                    should_wait=True)

            elif self.status.robot_status == "shoot":
                self.kick.shoot_ball()
            elif self.status.robot_status == "shoot_right":
                self.kick.shoot_ball(target="right")
            elif self.status.robot_status == "shoot_left":
                self.kick.shoot_ball(target="left")
            elif self.status.robot_status == "shoot_center":
                self.kick.shoot_ball(target="center")
            elif self.status.robot_status == "prepare_shoot":
                self.kick.shoot_ball(should_wait=True)

            elif self.status.robot_status == "penalty_shoot":
                self.kick.shoot_ball(ignore_penalty_area=True)
            elif self.status.robot_status == "penalty_shoot_right":
                self.kick.shoot_ball(target="right", ignore_penalty_area=True)
            elif self.status.robot_status == "penalty_shoot_left":
                self.kick.shoot_ball(target="left", ignore_penalty_area=True)
            elif self.status.robot_status == "penalty_shoot_center":
                self.kick.shoot_ball(target="center", ignore_penalty_area=True)

            elif self.status.robot_status == "receive":
                self.kick.receive_ball(self.ctrld_robot.get_future_position()[0],
                                       self.ctrld_robot.get_future_position()[1])
            elif self.status.robot_status == "receive_direct_pass":
                self.kick.receive_and_direct_pass(self.ctrld_robot.get_future_position(),
                                                  self.ctrld_robot.get_pass_target_position())
            elif self.status.robot_status == "receive_direct_shoot":
                self.kick.receive_and_direct_shoot(self.ctrld_robot.get_future_position())
            elif self.status.robot_status == "receive_direct_shoot_left":
                self.kick.receive_and_direct_shoot(self.ctrld_robot.get_future_position(), target="left")
            elif self.status.robot_status == "receive_direct_shoot_right":
                self.kick.receive_and_direct_shoot(self.ctrld_robot.get_future_position(), target="right")
            elif self.status.robot_status == "receive_direct_shoot_center":
                self.kick.receive_and_direct_shoot(self.ctrld_robot.get_future_position(), target="center")
            elif self.status.robot_status == "defence1":
                self.defence.move_defence(
                    self.defence.def1_pos_x, self.defence.def1_pos_y)
            elif self.status.robot_status == "defence2":
                self.defence.move_defence(
                    self.defence.def2_pos_x, self.defence.def2_pos_y)
            elif self.status.robot_status == "defence3":
                self.kick.receive_ball(
                    self.defence.def1_pos_x, self.defence.def1_pos_y)
            elif self.status.robot_status == "defence4":
                self.kick.receive_ball(
                    self.defence.def2_pos_x, self.defence.def2_pos_y)

            elif self.status.robot_status == "keeper":
                self.keeper.keeper()

            elif self.status.robot_status == "keeper_pass_chip":
                self.kick.pass_ball(self.ctrld_robot.get_pass_target_position()[0],
                                    self.ctrld_robot.get_pass_target_position()[1],
                                    is_tip_kick=True,
                                    ignore_penalty_area=True)

            elif self.status.robot_status == "stop":
                self.reset_cmd()
                self.status.robot_status = "none"
            elif self.status.robot_status == "halt":
                self.reset_cmd()

            elif self.status.robot_status == "shutdown":
                self.shutdown_cmd()

            self.store_and_publish_commands()
            loop_rate.sleep()

            #elapsed_time = time.time() - start
            #print ("elapsed_time:{0}".format(1./elapsed_time) + "[Hz]")

        """

    def goal_pose_listener(self):
        rospy.Subscriber("/robot_" + self.robot_id + "/goal_pose", Pose, self.ctrld_robot.goal_pose_callback)

    def target_pose_listener(self):
        rospy.Subscriber("/robot_" + self.robot_id + "/target_pose", Pose, self.ctrld_robot.target_pose_callback)

        """

    def status_listener(self):
        rospy.Subscriber("/" + self.robot_color + "/robot_" +
                         self.robot_id + "/status", Status, self.status.status_callback)

    def set_pid_server(self):
        rospy.Service("/" + self.robot_color + "/robot_" +
                      self.robot_id + "/set_pid", pid, self.pid.set_pid_callback)

    """
    def kick(self, req):
        self.cmd.kick_speed_x = 3
        return True

    def kick_end(self, req):
        self.cmd.kick_speed_x = 0
        return True

    def kick_server(self):
        rospy.Service("/robot_0/kick", Kick, self.kick)
        rospy.Service("/robot_0/kick_end", Kick, self.kick_end)
    """

    def def_pos_listener(self):
        rospy.Subscriber("/" + self.robot_color + "/def_pos",
                         Def_pos, self.defence.def_pos_callback)


def run_robot():
    robot = Robot()
    try:
        robot.run()
    except:
        import traceback
        traceback.print_exc()
        print("Robot color:" + robot.robot_color)
        print("Robot id   :" + robot.robot_id)


if __name__ == "__main__":
    while True and not rospy.is_shutdown():
        try:
            run_robot()
        except:
            import traceback
            traceback.print_exc()


"""
    robot.odom_listener()
    #robot.goal_pose_listener()
    #robot.goal_pose_listener()
    robot.status_listener()
    #robot.kick_server()
    loop_rate = rospy.Rate(ROBOT_LOOP_RATE)

    print("start")

    while not rospy.is_shutdown():
        if robot.status.robot_status == "move_linear":
            robot.pid.pid_linear(robot.ctrld_robot.goal_pos_x, robot.ctrld_robot.goal_pos_y,
            robot.ctrld_robot.goal_pos_theta)
        if robot.status.robot_status == "move_circle":
            robot.pid.pid_circle(robot.pid.pid_circle_center_x, robot.ctrld_robot.goal_pos_x,
            robot.ctrld_robot.goal_pos_y, robot.pid.pid_circle_center_y, robot.ctrld_robot.goal_pos_theta)
        if robot.status.robot_status == "kick":
            robot.kick.kick_x()
        if robot.status.robot_status == "pass":
            robot.kick.pass_ball(robot.ctrld_robot.target_pos_x, robot.ctrld_robot.target_pos_y)

        print(robot.status.robot_status)
        loop_rate.sleep()

"""
