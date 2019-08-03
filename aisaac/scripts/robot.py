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

from objects import Objects
from robot_command_publisher_wrapper import RobotCommandPublisherWrapper

import config

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

        self.cmd = robot_commands()
        self.cmd.kick_speed_x = 0
        self.cmd.kick_speed_z = 0
        self.cmd.dribble_power = 0

        self._command_pub = RobotCommandPublisherWrapper(self.robot_color, self.robot_id)

        # Composition
        self.objects = Objects(self.robot_color, self.robot_total, self.enemy_total)

        self.ctrld_robot = self.objects.robot[int(self.robot_id)]

        self.robot_friend = self.objects.robot
        self.robot_enemy = self.objects.enemy

        self.ball_params = self.objects.ball

        self.pid = RobotPid(self.robot_id, self.objects, self.cmd)
        self.status = RobotStatus(self.pid)
        self.kick = RobotKick(self.pid, self.cmd, self.status)
        self.defence = RobotDefence(self.status)
        self.keeper = RobotKeeper(self.kick)

        # listner 起動
        # self.odom_listener()
        # self.goal_pose_listener()
        # self.target_pose_listener()
        self.status_listener()
        self.set_pid_server()
        self.def_pos_listener()
        rospy.Timer(rospy.Duration(0.1), self.pid.replan_timer_callback)

    def run(self):
        # Loop 処理
        self.loop_rate = rospy.Rate(ROBOT_LOOP_RATE)
        rospy.loginfo("Robot start: "+self.robot_id)
        while not rospy.is_shutdown():
            # start = time.time()

            if self.status.robot_status == "move_linear":
                self.pid.pid_linear(self.ctrld_robot.get_future_position()[0],
                                    self.ctrld_robot.get_future_position()[1],
                                    self.ctrld_robot.get_future_orientation())
            elif self.status.robot_status == "move_circle":
                self.pid.pid_circle(self.pid.pid_circle_center_x, self.pid.pid_circle_center_y,
                                    self.ctrld_robot.get_future_position()[0],
                                    self.ctrld_robot.get_future_position()[1],
                                    self.ctrld_robot.get_future_orientation())
            elif self.status.robot_status == "kick":
                self.kick.kick_x()
            elif self.status.robot_status == "pass":
                self.kick.pass_ball(self.ctrld_robot.get_pass_target_position()[
                                    0], self.ctrld_robot.get_pass_target_position()[1])
            elif self.status.robot_status == "receive":
                self.kick.receive_ball(self.ctrld_robot.get_pass_target_position()[
                                       0], self.ctrld_robot.get_pass_target_position()[1])
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

            self.publish_commands()

            self.loop_rate.sleep()
            #elapsed_time = time.time() - start
            #print ("elapsed_time:{0}".format(1./elapsed_time) + "[Hz]")

    def publish_commands(self):
        self._command_pub.publish(self.cmd)

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


if __name__ == "__main__":
    is_single_thread = True

    if is_single_thread:
        robot = Robot()
        robot.run()
    else:
        import threading
        robots = []
        for i in range(config.NUM_FRIEND_ROBOT):
            robot = Robot(str(i))
            th = threading.Thread(target=robot.run)
            th.setDaemon(True)
            th.start()

        while not rospy.is_shutdown():
            pass


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
