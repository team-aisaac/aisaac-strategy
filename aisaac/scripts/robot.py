#!/usr/bin/env  python
# coding:utf-8
import math
import rospy
from consai_msgs.msg import Pose
from consai_msgs.msg import robot_commands
#from aisaac.srv import Kick
from aisaac.msg import Status
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
from robot_functions import RobotParameters, RobotPid, Ball, RobotStatus, RobotKick

ROBOT_LOOP_RATE = 60.

class Robot():
    def __init__(self):
        rospy.init_node("robot")
        self.robot_color = str(rospy.get_param("~robot_color"))
        self.robot_num = str(rospy.get_param("~robot_num"))

        self.friend_num = 8
        self.enemy_num = 8

        self.cmd = robot_commands()
        self.cmd.kick_speed_x = 0
        self.cmd.kick_speed_z = 0
        self.cmd.dribble_power = 0

        self.command_pub = rospy.Publisher("/" + self.robot_color + "/robot_" + self.robot_num + "/robot_commands", robot_commands, queue_size=10)

        # Composition
        self.robot_params = RobotParameters(self.robot_num,self.friend_num, self.enemy_num)
        self.ball_params = Ball()
        self.pid = RobotPid(self.robot_params, self.ball_params, self.cmd, self.command_pub)
        self.status = RobotStatus(self.pid, self.robot_params)
        self.kick = RobotKick(self.ball_params, self.robot_params, self.pid, self.cmd, self.status, self.command_pub)

        # listner 起動
        self.odom_listener()
        #self.goal_pose_listener()
        #self.target_pose_listener()
        self.status_listener()
        rospy.Timer(rospy.Duration(0.1), self.pid.replan_timerCallback)

        #Loop 処理
        self.loop_rate = rospy.Rate(ROBOT_LOOP_RATE)
        while not rospy.is_shutdown():
            if self.status.robot_status == "move_linear":
                self.pid.pid_linear(self.robot_params.pid_goal_pos_x, self.robot_params.pid_goal_pos_y,
                self.robot_params.pid_goal_theta)
            if self.status.robot_status == "move_circle":
                self.pid.pid_circle(self.pid.pid_circle_center_x, self.pid.pid_circle_center_y,
                self.robot_params.pid_goal_pos_x, self.robot_params.pid_goal_pos_y, self.robot_params.pid_goal_theta)
            if self.status.robot_status == "kick":
                self.kick.kick_x()
            if self.status.robot_status == "pass":
                self.kick.pass_ball(self.robot_params.pass_target_pos_x, self.robot_params.pass_target_pos_y)
            if self.status.robot_status == "receive":
                self.kick.recieve_ball(self.robot_params.pass_target_pos_x,self.robot_params.pass_target_pos_y)
            self.loop_rate.sleep()
            

    def odom_listener(self):
        for i in range(self.friend_num):
            rospy.Subscriber("/" + self.robot_color + "/robot_" + str(i) + "/odom", Odometry, self.robot_params.friend_odom_callback, callback_args=i)
        for j in range(self.enemy_num):
            rospy.Subscriber("/" + self.robot_color + "/enemy_" + str(j) + "/odom", Odometry, self.robot_params.enemy_odom_callback, callback_args=j)
        rospy.Subscriber("/" + self.robot_color + "/ball_observer/estimation", Odometry, self.ball_params.odom_callback)

        """

    def goal_pose_listener(self):
        rospy.Subscriber("/robot_" + self.robot_num + "/goal_pose", Pose, self.robot_params.goal_pose_callback)

    def target_pose_listener(self):
        rospy.Subscriber("/robot_" + self.robot_num + "/target_pose", Pose, self.robot_params.target_pose_callback)

        """

    def status_listener(self):
        rospy.Subscriber("/" + self.robot_color + "/robot_" + self.robot_num + "/status", Status, self.status.status_callback)

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


if __name__ == "__main__":
    robot = Robot()
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
            robot.pid.pid_linear(robot.robot_params.goal_pos_x, robot.robot_params.goal_pos_y,
            robot.robot_params.goal_pos_theta)
        if robot.status.robot_status == "move_circle":
            robot.pid.pid_circle(robot.pid.pid_circle_center_x, robot.robot_params.goal_pos_x,
            robot.robot_params.goal_pos_y, robot.pid.pid_circle_center_y, robot.robot_params.goal_pos_theta)
        if robot.status.robot_status == "kick":
            robot.kick.kick_x()
        if robot.status.robot_status == "pass":
            robot.kick.pass_ball(robot.robot_params.target_pos_x, robot.robot_params.target_pos_y)

        print(robot.status.robot_status)
        loop_rate.sleep()
    
"""