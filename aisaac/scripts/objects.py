#!/usr/bin/env  python
# coding:utf-8
import entity
import rospy
from nav_msgs.msg import Odometry
import tf


class Objects(object):
    def __init__(self, team_color, robot_total, enemy_total):
        self.team_color = team_color
        self.robot_total = robot_total
        self.enemy_total = enemy_total
        self.robot = [entity.Robot() for i in range(self.robot_total)]
        self.enemy = [entity.Robot() for i in range(self.enemy_total)]
        self.ball = entity.Ball()

        """---ボール軌道の考慮時間幅(linear Regressionで軌道予測するため)---"""
        self.ball_dynamics_window = 5
        self.ball_dynamics = [[0., 0.] for i in range(self.ball_dynamics_window)]

        self.odom_listener()

    def set_first_positions(self):
        self.robot[0].set_future_position(x=-5.5, y=0., theta=0.)
        self.robot[1].set_future_position(x=-4.5, y=1., theta=0.)
        self.robot[2].set_future_position(x=-4.5, y=-1., theta=0.)
        self.robot[3].set_future_position(x=-2., y=2.5, theta=0.)
        self.robot[4].set_future_position(x=-2., y=-2.5, theta=0.)
        self.robot[5].set_future_position(x=1.5, y=2.5, theta=0.)
        self.robot[6].set_future_position(x=1.5, y=-2.5, theta=0.)
        self.robot[7].set_future_position(x=2.5, y=0., theta=0.)

    """---4台用守備時(20181216練習会用に作られた)---"""
    def set_first_positions_4robots(self):
        self.robot[0].set_future_position(x=-5.5, y=0., theta=0.)
        self.robot[1].set_future_position(x=-4.5, y=1., theta=0.)
        self.robot[2].set_future_position(x=-4.5, y=-1., theta=0.)
        self.robot[3].set_future_position(x=-2., y=0., theta=0.)
        #self.robot[4].set_future_position(x=-5.5, y=0., theta=0.)
        #self.robot[5].set_future_position(x=-4.5, y=5., theta=0.)
        #self.robot[6].set_future_position(x=-4., y=5., theta=0.)
        #self.robot[7].set_future_position(x=-3.5, y=5., theta=0.)

    """---4台用攻撃時(20181216練習会用に作られた)---"""
    def set_first_position_4robots_attack(self):
        self.robot[0].set_future_position(x=-5.5, y=0., theta=0.)
        self.robot[1].set_future_position(x=-4.5, y=1., theta=0.)
        self.robot[2].set_future_position(x=-4.5, y=-1., theta=0.)
        self.robot[3].set_future_position(x=0.-self.robot[3].robot_r, y=0., theta=0.)
        #self.robot[4].set_future_position(x=-5.5, y=0., theta=0.)
        #self.robot[5].set_future_position(x=-4.5, y=5., theta=0.)
        #self.robot[6].set_future_position(x=-4., y=5., theta=0.)
        #self.robot[7].set_future_position(x=-3.5, y=5., theta=0.)

    """---Visionから現在地をもらうsubscriberの起動--"""
    def odom_listener(self):
        for i in range(self.robot_total):
            rospy.Subscriber("/" + self.team_color + "/robot_"+ str(i) +"/odom", Odometry, self.robot_odom_callback, callback_args=i)

        for j in range(self.enemy_total):
            rospy.Subscriber("/" + self.team_color + "/enemy_" + str(j) + "/odom", Odometry, self.enemy_odom_callback, callback_args=j)

        rospy.Subscriber("/" + self.team_color + "/ball_observer/estimation", Odometry, self.ball_odom_callback)


    """---Visionからrobotの現在地をもらう---"""
    def robot_odom_callback(self, msg, id):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        robot_v_x = msg.twist.twist.linear.x;
        robot_v_y = msg.twist.twist.linear.y;
        robot_v_t = msg.twist.twist.linear.z;
        self.robot[id].set_current_position(x = robot_x, y = robot_y, theta=robot_t[2])
        self.robot[id].set_current_velocity(vx = robot_v_x, vy = robot_v_y, vtheta=robot_v_t)

    """---Visionからenemyの現在地をもらう---"""
    def enemy_odom_callback(self, msg, id):
        enemy_x = msg.pose.pose.position.x
        enemy_y = msg.pose.pose.position.y
        enemy_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        enemy_v_x = msg.twist.twist.linear.x;
        enemy_v_y = msg.twist.twist.linear.y;
        enemy_v_t = msg.twist.twist.linear.z;
        self.enemy[id].set_current_position(x = enemy_x, y = enemy_y, theta=enemy_t[2])
        self.enemy[id].set_current_velocity(vx = enemy_v_x, vy = enemy_v_y, vtheta=enemy_v_t)

    """---Visionからballの現在地をもらう---"""
    def ball_odom_callback(self, msg):
        ball_x = msg.pose.pose.position.x
        ball_y = msg.pose.pose.position.y
        ball_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.ball.set_current_position(x = ball_x, y = ball_y, theta=ball_t[2])
        _ = self.ball_dynamics.pop(0)
        self.ball_dynamics.append([ball_x, ball_y])
