#!/usr/bin/env  python
# coding:utf-8
import entity
import rospy
from nav_msgs.msg import Odometry
from aisaac.msg import Ball_sub_params
import tf
import functions


class Objects(object):

    # シングルトン化
    __instance = None
    def __new__(cls, *args, **keys):
        if cls.__instance is None:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def __init__(self, team_color, robot_total, enemy_total):
        # type: (str, int, int) -> None
        self.team_color = team_color
        self.robot_total = robot_total
        self.enemy_total = enemy_total

        self._robot_ids = range(self.robot_total)
        self._enemy_ids = range(self.enemy_total)

        self.robot = [entity.Robot(id=i) for i in self._robot_ids]  # type: typing.List[entity.Robot]
        self.enemy = [entity.Robot() for i in self._enemy_ids]  # type: typing.List[entity.Robot]

        roles = ["RFW", "LFW", "RDF", "LDF", "GK"]
        for robot, role in zip(self.robot, roles):
            robot.set_role(role)

        self.ball = entity.Ball()

        """---ボール軌道の考慮時間幅(linear Regressionで軌道予測するため)---"""
        self.ball_dynamics_window = 5
        self.ball_dynamics = [[0., 0.] for i in range(self.ball_dynamics_window)]

        self.odom_listener()

    def get_robot_ids_sorted_by_distance_to_ball(self, robot_ids=None):
        return self.get_robot_ids_sorted_by_distance(self.ball.get_current_position(), robot_ids)

    def get_robot_ids_sorted_by_distance(self, target_xy, robot_ids=None):
        target_x = target_xy[0]
        target_y = target_xy[1]

        if robot_ids is None:
            robots = self.robot
        else:
            robots = [self.robot[i] for i in robot_ids]

        sorted_robots = sorted(robots,
                               key=lambda robot: functions.distance_btw_two_points(robot.get_current_position(),
                                                                                   (target_x, target_y)))
        sorted_ids = map(lambda robot: robot.get_id(), sorted_robots)
        return sorted_ids

    def get_robot_ids(self):
        return self._robot_ids

    def get_enemy_ids(self):
        return self._enemy_ids

    def get_robot_by_id(self, robot_id):
        robots = [self.robot[i] for i in self._robot_ids]
        for i, id in enumerate(self._robot_ids):
            if id == robot_id:
                return robots[i]

    def get_robot_by_role(self, role):
        robots = [self.robot[i] for i in self._robot_ids]
        for i, robot in enumerate(robots):
            if robot.get_role() == role:
                return robots[i]

    def get_robot_id_by_role(self, role):
        robots = [self.robot[i] for i in self._robot_ids]
        for i, robot in enumerate(robots):
            if robot.get_role() == role:
                return robot.get_id()

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
        rospy.Subscriber("/" + self.team_color + "/ball_sub_params", Ball_sub_params, self.ball_sub_params_callback)


    """---Visionからrobotの現在地をもらう---"""
    def robot_odom_callback(self, msg, id):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2]
        robot_v_x = msg.twist.twist.linear.x
        robot_v_y = msg.twist.twist.linear.y
        robot_v_t = msg.twist.twist.linear.z
        self.robot[id].set_vision_position(x = robot_x, y = robot_y, theta=robot_t)
        self.robot[id].set_current_velocity(vx = robot_v_x, vy = robot_v_y, vtheta=robot_v_t)

    """---Visionからenemyの現在地をもらう---"""
    def enemy_odom_callback(self, msg, id):
        enemy_x = msg.pose.pose.position.x
        enemy_y = msg.pose.pose.position.y
        enemy_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2]
        enemy_v_x = msg.twist.twist.linear.x
        enemy_v_y = msg.twist.twist.linear.y
        enemy_v_t = msg.twist.twist.linear.z
        self.enemy[id].set_vision_position(x = enemy_x, y = enemy_y, theta=enemy_t)
        self.enemy[id].set_current_velocity(vx = enemy_v_x, vy = enemy_v_y, vtheta=enemy_v_t)

    """---Visionからballの現在地をもらう---"""
    def ball_odom_callback(self, msg):
        ball_x = msg.pose.pose.position.x
        ball_y = msg.pose.pose.position.y
        ball_t = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2]
        self.ball.set_current_position(x = ball_x, y = ball_y, theta=ball_t)
        _ = self.ball_dynamics.pop(0)
        self.ball_dynamics.append([ball_x, ball_y])

    def ball_sub_params_callback(self, msg):
        self.ball.set_line_a(msg.a)
        self.ball.set_line_b(msg.b)
        self.ball.set_future_position(msg.future_x, msg.future_y)
