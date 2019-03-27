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

from robot_functions import RobotParameters, RobotMove, Ball, RobotStatus, RobotKick

ROBOT_LOOP_RATE = 60.

class Robot():
    def __init__(self):
        rospy.init_node("robot")
        self.robot_color = str(rospy.get_param("~robot_color"))
        self.robot_num = str(rospy.get_param("~robot_num"))

        self.cmd = robot_commands()
        self.cmd.kick_speed_x = 0
        self.cmd.kick_speed_z = 0
        self.cmd.dribble_power = 0

        self.command_pub = rospy.Publisher("/" + self.robot_color + "/robot_" + self.robot_num + "/robot_commands", robot_commands, queue_size=10)

        # Composition
        self.robot_params = RobotParameters(self.robot_num)
        self.move = RobotMove(self.robot_params, self.cmd, self.command_pub)
        self.ball_params = Ball()
        self.status = RobotStatus(self.move, self.robot_params)
        self.kick = RobotKick(self.ball_params, self.robot_params, self.move, self.cmd, self.status, self.command_pub)

        # listner 起動
        self.odom_listener()
        #self.goal_pose_listener()
        #self.target_pose_listener()
        self.status_listener()

        #Loop 処理
        self.loop_rate = rospy.Rate(ROBOT_LOOP_RATE)
        while not rospy.is_shutdown():
            if self.status.robot_status == "move_linear":
                self.move.pid_linear(self.robot_params.pid_goal_pos_x, self.robot_params.pid_goal_pos_y,
                self.robot_params.pid_goal_theta)
            elif self.status.robot_status == "move_circle":
                self.move.pid_circle(self.pid.pid_circle_center_x, self.pid.pid_circle_center_y,
                self.robot_params.pid_goal_pos_x, self.robot_params.pid_goal_pos_y, self.robot_params.pid_goal_theta)
            elif self.status.robot_status == "kick":
                self.kick.kick_x()
            elif self.status.robot_status == "pass":
                self.kick.pass_ball(self.robot_params.pass_target_pos_x, self.robot_params.pass_target_pos_y)
            elif self.status.robot_status == "stop":
                self.move.stop()
            #print(self.robot_num, self.status.robot_status)
            self.loop_rate.sleep()


    def odom_listener(self):
    	rospy.Subscriber("/" + self.robot_color + "/robot_" + self.robot_num + "/odom", Odometry, self.robot_params.odom_callback)
        rospy.Subscriber("/" + self.robot_color + "/ball_observer/estimation", Odometry, self.ball_params.odom_callback)

    def status_listener(self):
        rospy.Subscriber("/" + self.robot_color + "/robot_" + self.robot_num + "/status", Status, self.status.status_callback)


if __name__ == "__main__":
    robot = Robot()
