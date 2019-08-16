#!/usr/bin/env  python
# coding:utf-8
import entity
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16MultiArray
from aisaac.msg import Ball_sub_params
import tf
import functions
import config
import copy


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

        self._active_robot_ids = range(self.robot_total)
        self._active_enemy_ids = range(self.robot_total)

        self.robot = [entity.Robot(id=i) for i in self._robot_ids]  # type: typing.List[entity.Robot]
        self.enemy = [entity.Robot(id=i) for i in self._enemy_ids]  # type: typing.List[entity.Robot]

        # roles = ["RFW", "LFW", "RDF", "LDF", "GK"]
        # for robot, role in zip(self.robot, roles):
        #     robot.set_role(role)

        roles = ["RFW", "LFW", "RDF", "LDF", "GK"]
        for robot_id, role in zip(self._robot_ids, roles):
            self.get_robot_by_id(robot_id).set_role(role)

        rospy.Timer(rospy.Duration(5.0), self.redefine_roles)

        self.ball = entity.Ball()

        """---ボール軌道の考慮時間幅(linear Regressionで軌道予測するため)---"""
        self.ball_dynamics_window = 5
        self.ball_dynamics = [[0., 0.] for i in range(self.ball_dynamics_window)]

        self.odom_listener()
        self.existing_listener()

    def redefine_roles(self, event):
        if len(self._active_robot_ids) == 5:
            roles = ["RFW", "LFW", "RDF", "LDF", "GK"]
        elif len(self._active_robot_ids) == 4:
            roles = ["RFW", "LFW", "LDF", "GK"]
        elif len(self._active_robot_ids) == 3:
            roles = ["RFW", "LDF", "GK"]
        elif len(self._active_robot_ids) == 2:
            roles = ["RFW", "GK"]
        elif len(self._active_robot_ids) == 1:
            roles = ["RFW"]
        else:
            roles = ["RFW"]
            print ("error")

        for robot_id, role in zip(self._active_robot_ids, roles):
            self.get_robot_by_id(robot_id).set_role(role)


    def get_robot_ids_sorted_by_distance_to_ball(self, robot_ids=None):
        # type: (typing.List[int]) -> typing.List[int]
        return self.get_robot_ids_sorted_by_distance(self.ball.get_current_position(), robot_ids)

    def get_enemy_ids_sorted_by_distance_to_ball(self, enemy_ids=None):
        return self.get_enemy_ids_sorted_by_distance(self.ball.get_current_position(), enemy_ids)

    def get_robot_ids_sorted_by_distance(self, target_xy, robot_ids=None):
        # type: (typing.List[float], typing.List[int]) -> typing.List[int]
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

    def get_enemy_ids_sorted_by_distance(self, target_xy, enemy_ids=None):
        target_x = target_xy[0]
        target_y = target_xy[1]

        if enemy_ids is None:
            enemys = self.enemy
        else:
            enemys = [self.enemy[i] for i in enemy_ids]

        sorted_enemys = sorted(enemys,
                               key=lambda enemy: functions.distance_btw_two_points(enemy.get_current_position(),
                                                                                   (target_x, target_y)))
        sorted_ids = map(lambda enemy: enemy.get_id(), sorted_enemys)
        return sorted_ids

    def get_robot_ids(self):
        return self._robot_ids

    def get_enemy_ids(self):
        return self._enemy_ids

    def get_robot_by_id(self, robot_id):
        for robot in self.robot:
            if robot.get_id() == robot_id:
                return robot

    def get_enemy_by_id(self, enemy_id):
        for enemy in self.enemy:
            if enemy.get_id() == enemy_id:
                return enemy

    def get_robot_by_role(self, role):
        for robot in self.robot:
            if robot.get_role() == role:
                return robot

    def get_robot_id_by_role(self, role):
        for robot in self.robot:
            if robot.get_role() == role:
                return robot.get_id()

    def get_active_robot_ids(self):
        return copy.deepcopy(self._active_robot_ids)
        # return copy.deepcopy(self._robot_ids)

    def get_active_enemy_ids(self):
        return copy.deepcopy(self._active_enemy_ids)

    def get_ball_in_penalty_area(self):
        return functions.in_penalty_area(self.ball.get_current_position())

    def get_has_a_ball(self, robot_id, threshold=None):
        robot_ids = self.get_active_robot_ids()
        sorted_ids = self.get_robot_ids_sorted_by_distance_to_ball(robot_ids)
        if sorted_ids[0] != robot_id:
            return False

        if threshold is None:
            threshold = config.HAS_A_BALL_DISTANCE_THRESHOLD

        area = threshold + self.robot[0].size_r
        if functions.distance_btw_two_points(
                self.robot[robot_id].get_current_position(), self.ball.get_current_position()) \
                > area:
            return False

        return True

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

    """---Existingしているrobotの情報を獲得--"""
    def existing_listener(self):
        rospy.Subscriber("/" + self.team_color + "/existing_friends_id", UInt16MultiArray, self.existing_friends_callback)
        rospy.Subscriber("/" + self.team_color + "/existing_enemies_id", UInt16MultiArray, self.existing_enemies_callback)

    def existing_friends_callback(self, msg):
        self._active_robot_ids = [int(id) for id in msg.data]

    def existing_enemies_callback(self, msg):
        self._active_enemy_ids = [int(id) for id in msg.data]
