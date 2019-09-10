#!/usr/bin/env  python
# coding:utf-8
import numpy as np
import math
import matplotlib.pyplot as plt
import functions
import rospy
import functions
import config


class RobotKick(object):
    def __init__(self, pid, cmd, status):
        # type: (robot_pid.RobotPid, aisaac.msg.Status, robot_status.RobotStatus) -> None
        self.kick_power_x = 10
        self.kick_power_z = 0
        self.dribble_power = 10

        self.ball_params = pid.ball_params
        self.ctrld_robot = pid.ctrld_robot
        self.pid = pid
        self.status = status
        self.cmd = cmd
        self.dispersion1 = [10] * 1
        self.dispersion2 = [10] * 1
        self.rot_dispersion = [10] * 1

        self.kick_access_threshold1 = 0.1
        self.kick_access_threshold2 = 0.1
        self.kick_feint_threshold1 = 0.2
        self.kick_feint_threshold2 = 0.2
        self.kick_rot_access_threshold = 0.1
        self.dribble_access_threshold1 = 0.03
        self.dribble_access_threshold2 = 0.05
        self.dribble_rot_access_threshold = 0.1
        self.pass_stage = 0
        self.dribble_stage = 0
        self._kick_start_time = rospy.Time.now()
        self._dribble_start_pos = self.ball_params.get_current_position()

        self.const = 3.0
        self.const_place = 2.0
        # self.const = 4

        self.ball_frame = 60 #ボールの軌道の直線フィッティングと速度の計算フレーム数、使ってない
        self.ball_pos_x_array = np.array([0.0]*self.ball_frame) #グラフ描画用配列
        self.ball_pos_y_array = np.array([0.0]*self.ball_frame) #グラフ描画用配列
        #self.reach_flag = False #到達フラグ、使ってない

        self.plot_x = np.arange(-7.0,7.0, 0.01) #グラフ描画用配列
        self.plot_y = np.arange(-7.0,7.0, 0.01) #グラフ描画用配列
        self.fig, self.ax = plt.subplots(1, 1) #グラフ描画用
        # 初期化的に一度plotしなければならない
        # そのときplotしたオブジェクトを受け取る受け取る必要がある．
        # listが返ってくるので，注意
        self.lines1, = self.ax.plot(self.ball_pos_x_array, self.ball_pos_y_array)
        self.lines2, = self.ax.plot(self.plot_x, self.plot_y)
        self.lines3, = self.ax.plot(self.plot_x, self.plot_y)
        self.lines4, = self.ax.plot(self.plot_x, self.plot_y)
        self.ax.set_xlim(-7, 7)
        self.ax.set_ylim(-7, 7)

    def kick_xz(self, power_x=10.0, power_z=0.0, ignore_penalty_area=False):
        area = 1.0
        elapsed_time = rospy.Time.now() - self._kick_start_time
        if functions.distance_btw_two_points(self.ball_params.get_current_position(),
                                             self.ctrld_robot.get_current_position()) \
                > self.ctrld_robot.size_r + area \
                or elapsed_time.to_sec() > 5.0 \
                or functions.distance_btw_two_points(self.ball_params.get_current_position(), self._dribble_start_pos) > 0.8:
            self.cmd.vel_surge = 0
            self.cmd.vel_sway = 0
            self.cmd.omega = 0
            self.cmd.kick_speed_x = 0
            self.status.robot_status = "None"
            self.pass_stage = 0
            return

        self.cmd.kick_speed_x = power_x
        self.cmd.kick_speed_z = power_z
        self.pid.pid_linear(self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], self.pose_theta, ignore_penalty_area=ignore_penalty_area)
        #self.cmd.vel_surge = 3

    def shoot_ball(self, should_wait=False, target="random", ignore_penalty_area=False):
        """
        Parameters
        ----------
        should_wait: boolean TrueならKick準備完了後、Kick直前でKickせず待機
        target:      str     "ramdom"ならleft/right/centerからランダムで選択、それ以外ならleft/right/centerにシュート
        """
        _candidates = ["left", "right", "center"]

        if target in _candidates:
            _target = target
        else:
            _target = np.random.choice(_candidates)

        post_offset = 0.2

        if _target == "center":
            self.pass_ball(config.GOAL_ENEMY_CENTER[0], config.GOAL_ENEMY_CENTER[1], should_wait=should_wait, is_shoot=True, ignore_penalty_area=ignore_penalty_area)
        elif _target == "left":
            self.pass_ball(config.GOAL_ENEMY_LEFT[0], config.GOAL_ENEMY_LEFT[1] - post_offset, should_wait=should_wait, is_shoot=True, ignore_penalty_area=ignore_penalty_area)
        elif _target == "right":
            self.pass_ball(config.GOAL_ENEMY_RIGHT[0], config.GOAL_ENEMY_RIGHT[1] + post_offset, should_wait=should_wait, is_shoot=True, ignore_penalty_area=ignore_penalty_area)

    

    def pass_ball(self, target_x, target_y, should_wait=False, is_shoot=False, is_tip_kick=False, ignore_penalty_area=False, place=False):
        """
        Parameters
        ----------
        should_wait: boolean TrueならKick準備完了後、Kick直前でKickせず待機
        is_shoot:    boolean Trueならshootの威力でKick、Falseならpassの威力でKick
        is_tip_kick  boolean Trueならtip kick
        place        boolean Trueなら目標位置でballが静止するようにKick
        """
        distance = math.sqrt((target_x - self.ball_params.get_current_position()[0])**2 + (target_y - self.ball_params.get_current_position()[1])**2)
        if distance != 0:
            #print self.pass_stage
            if self.pass_stage == 0:
                prepare_offset = 0.4
                pose_x = (- prepare_offset * target_x + (prepare_offset + distance) * self.ball_params.get_current_position()[0]) / distance
                pose_y = (- prepare_offset * target_y + (prepare_offset + distance) * self.ball_params.get_current_position()[1]) / distance
                pose_theta = math.atan2( (target_y - self.ctrld_robot.get_current_position()[1]) , (target_x - self.ctrld_robot.get_current_position()[0]) )

                a, b, c = functions.line_parameters(self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], target_x, target_y)

                self.dispersion1.append(
                        functions.distance_of_a_point_and_a_straight_line(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], a, b, c))
                        #(pose_x - self.ctrld_robot.get_current_position()[0])**2 \
                        #+ (pose_y - self.ctrld_robot.get_current_position()[1])**2)
                del self.dispersion1[0]

                self.dispersion2.append(
                        (pose_x - self.ctrld_robot.get_current_position()[0])**2 \
                        + (pose_y - self.ctrld_robot.get_current_position()[1])**2)
                del self.dispersion2[0]

                dispersion_average1 = sum(self.dispersion1)/len(self.dispersion1)
                dispersion_average2 = sum(self.dispersion2)/len(self.dispersion2)

                if dispersion_average1 > self.kick_feint_threshold1 \
                        or dispersion_average2 > self.kick_feint_threshold2 \
                        or should_wait:
                    pose_theta += np.pi/3.

                self.rot_dispersion.append((pose_theta - self.ctrld_robot.get_current_orientation())**2)
                del self.rot_dispersion[0]
                rot_dispersion_average = sum(self.rot_dispersion)/len(self.rot_dispersion)

                # print("{} {}".format(dispersion_average, rot_dispersion_average))

                if dispersion_average1 < self.kick_access_threshold1 and dispersion_average2 < self.kick_access_threshold2 and rot_dispersion_average < self.kick_rot_access_threshold:
                    self.pose_theta = pose_theta
                    # self.status.robot_status = "kick"
                    if not should_wait:
                        self._kick_start_time = rospy.Time.now()
                        self._dribble_start_pos = self.ball_params.get_current_position()
                        self.pass_stage = 1

                self.pid.pid_linear(pose_x, pose_y, pose_theta, ignore_penalty_area=ignore_penalty_area)

            elif self.pass_stage == 1:
                if not is_shoot:
                    kick_power_x = math.sqrt(distance) * self.const
                else:
                    # 12.0: フィールドの横幅
                    kick_power_x = math.sqrt(12.0) * self.const
                if is_tip_kick:
                    kick_power_z = math.sqrt(distance) * self.const
                    self.kick_xz(power_x=kick_power_x, power_z=kick_power_z, ignore_penalty_area=ignore_penalty_area)
                else:
                    if place:
                        kick_power_x = math.sqrt(distance) * self.const_place
                    self.kick_xz(power_x=kick_power_x, ignore_penalty_area=ignore_penalty_area)

            """
            if self.pass_stage == 1:
                self.kick_power_x = math.sqrt(distance) * self.const
                self.pose_theta = pose_theta
                self.status.robot_status = "kick"
            """

    def reg1dim(self, x, y, n):
        x = np.clip(x,-6.5,6.5)
        y = np.clip(y,-5.5,5.5)

        a = ((np.dot(x, y)- y.sum() * x.sum()/n) / ((x ** 2).sum() - x.sum()**2 / n))
        b = (y.sum() - a * x.sum())/n
        a = np.clip(a,-1.0e+308,1.0e+308)
        b = np.clip(b,-1.0e+308,1.0e+308)
        return a, b

    def receive_ball(self, target_x, target_y):
        self._recieve_ball((target_x, target_y),
                           self.ball_params.get_current_position())

    def receive_and_direct_shoot(self, target_xy, target="random"):
        _candidates = ["left", "right", "center"]

        if target in _candidates:
            _target = target

        else:
            _target = np.random.choice(_candidates)

        post_offset = 0.2

        # 位置調整
        vec_x = config.GOAL_ENEMY_CENTER[0] - target_xy[0]
        vec_y = config.GOAL_ENEMY_CENTER[1] - target_xy[1]
        len = np.sqrt(vec_x ** 2 + vec_y ** 2)
        target_xy = [target_xy[0] - (vec_x / len) * 0.09, target_xy[1] - (vec_y / len) * 0.09]

        if _target == "center":
            self._recieve_ball(target_xy, config.GOAL_ENEMY_CENTER, auto_kick=True, is_shoot=True)
        elif _target == "left":
            goal_enemy_left = [config.GOAL_ENEMY_LEFT[0], config.GOAL_ENEMY_LEFT[1] - post_offset]
            self._recieve_ball(target_xy, goal_enemy_left, auto_kick=True, is_shoot=True)
        elif _target == "right":
            goal_enemy_right = [config.GOAL_ENEMY_RIGHT[0], config.GOAL_ENEMY_RIGHT[1] + post_offset]
            self._recieve_ball(target_xy, goal_enemy_right, auto_kick=True, is_shoot=True)

    def receive_and_direct_pass(self, target_xy, next_target_xy):
        self._recieve_ball(target_xy,
                           next_target_xy,
                           auto_kick=True, is_shoot=False)

    def receive_and_clear(self, target_xy):
        self._recieve_ball(target_xy,
                           self.ball_params.get_current_position(),
                           auto_kick=True, is_shoot=False, is_tip_kick=True)

    def receive_ball_keeper(self, target_xy):
        self._recieve_ball(target_xy,
                           self.ball_params.get_current_position(),
                           auto_kick=True, is_shoot=True, ignore_penalty_area=True, is_tip_kick=True)

    def _recieve_ball(self, target_xy, next_target_xy, auto_kick=False, is_shoot=False, ignore_penalty_area=False, is_tip_kick=False):
        # type: (typing.Tuple[float], typing.Tuple[float]) -> None
        """
        Parameters
        ----------
        target_xy: (x, y)       パス目標地点
        next_target_xy: (x, y)  受け取り時にロボットが向いているべき方向
        auto_kick: boolean      Trueならキッカーに当たったらそのままキック、Falseなら受けるだけ
        is_shoot: boolean       auto_kickがTrueのときにのみ利用され、Trueならshootの威力でキック、Falseならpassの威力でキック
        ignore_penalty_area: boolean Trueならペナルティエリアに進入する、Falseなら進入しない
        """

        self.cmd.kick_speed_x = 0
        self.cmd.kick_speed_z = 0

        target_x = target_xy[0]
        target_y = target_xy[1]

        line_a = self.ball_params.get_line_a()
        line_b = self.ball_params.get_line_b()
        # 本来のパス目標地点とフィッティング直線Lとの距離計算
        d = abs(line_a*target_x - target_y + line_b) \
            / ((line_a**2 + 1)**(1/2))  # ヘッセの公式で距離計算
        # 交点H(hx, hy) の座標計算
        hx = (line_a * (target_y - line_b) + target_x) \
            / (line_a**2 + 1)
        hy = line_a * (line_a * (target_y - line_b) + target_x) \
            / (line_a**2 + 1) + line_b

        # TODO: 機体の速度・加速度から間に合うかどうか判断

        # 距離だけで諦めるかどうか判断
        if (d < 2.0) and  (line_a != 0):
            pose_theta = math.atan2( (next_target_xy[1] - hy) , (next_target_xy[0] - hx) )
            # pose_theta = math.atan2( (next_target_xy[1]) , (next_target_xy[0]) )
            self.pid.pid_linear(hx, hy, pose_theta, ignore_penalty_area=ignore_penalty_area)
        else:
            pose_theta = math.atan2( (next_target_xy[1] - target_y) , (next_target_xy[0] - target_x) )
            # pose_theta = math.atan2( (next_target_xy[1]) , (next_target_xy[0]) )
            self.pid.pid_linear(target_x, target_y, pose_theta, ignore_penalty_area=ignore_penalty_area)

        if auto_kick:
            distance = functions.distance_btw_two_points((target_x, target_y), next_target_xy)
            if not is_shoot:
                kick_power_x = math.sqrt(distance) * self.const
            else:
                # 12.0: フィールドの横幅
                kick_power_x = math.sqrt(12.0) * self.const

            if is_tip_kick:
                kick_power_x = math.sqrt(12.0) * self.const
                self.cmd.kick_speed_z = kick_power_x

            self.cmd.kick_speed_x = kick_power_x

        # # 垂線テキスト座標
        # if self.ctrld_robot.get_id() == 0:
        #     dx_center = (target_x + hx) / 2
        #     dy_center = (target_y + hy) / 2
        #     plt.axis('scaled')
        #     #plt.plot([target_x, hx],[target_y, hy], color='green', linestyle='--', zorder=0)

        #     self.plot_y = line_a * self.plot_x + line_b
        #     self.lines1.set_data(self.ball_pos_x_array, self.ball_pos_y_array)
        #     self.lines2.set_data(self.plot_x, self.plot_y)
        #     self.lines3.set_data([target_x, hx], [target_y, hy])
        #     #self.lines3.set_data([self.ball_params.ball_future_x, hx], [self.ball_params.ball_future_y, hy])
        #     #self.lines4.set_data([self.ball_params.ball_future_x, target_x], [self.ball_params.ball_future_y, target_y])
        #     plt.pause(.01)

    def dribble_ball(self, target_x, target_y, ignore_penalty_area=False):
        distance = math.sqrt((target_x - self.ball_params.get_current_position()[0])**2 + (target_y - self.ball_params.get_current_position()[1])**2)
        if distance != 0:
            #ボール後方へ移動
            if self.dribble_stage == 0:
                prepare_offset = 0.4
                pose_x = (- prepare_offset * target_x + (prepare_offset + distance) * self.ball_params.get_current_position()[0]) / distance
                pose_y = (- prepare_offset * target_y + (prepare_offset + distance) * self.ball_params.get_current_position()[1]) / distance
                pose_theta = math.atan2( (target_y - self.ctrld_robot.get_current_position()[1]) , (target_x - self.ctrld_robot.get_current_position()[0]) )

                a, b, c = functions.line_parameters(self.ball_params.get_current_position()[0], self.ball_params.get_current_position()[1], target_x, target_y)

                self.dispersion1.append(
                        functions.distance_of_a_point_and_a_straight_line(self.ctrld_robot.get_current_position()[0], self.ctrld_robot.get_current_position()[1], a, b, c))
                        #(pose_x - self.ctrld_robot.get_current_position()[0])**2 \
                        #+ (pose_y - self.ctrld_robot.get_current_position()[1])**2)
                del self.dispersion1[0]

                self.dispersion2.append(
                        (pose_x - self.ctrld_robot.get_current_position()[0])**2 \
                        + (pose_y - self.ctrld_robot.get_current_position()[1])**2)
                del self.dispersion2[0]

                dispersion_average1 = sum(self.dispersion1)/len(self.dispersion1)
                dispersion_average2 = sum(self.dispersion2)/len(self.dispersion2)

                self.rot_dispersion.append((pose_theta - self.ctrld_robot.get_current_orientation())**2)
                del self.rot_dispersion[0]
                rot_dispersion_average = sum(self.rot_dispersion)/len(self.rot_dispersion)

                # print("{} {}".format(dispersion_average, rot_dispersion_average))

                if dispersion_average1 < self.dribble_access_threshold1 and dispersion_average2 < self.dribble_access_threshold2 and rot_dispersion_average < self.dribble_rot_access_threshold:
                    self.dribble_stage = 1

                self.pid.pid_linear(pose_x, pose_y, pose_theta, ignore_penalty_area=ignore_penalty_area)

            #目標地付近までドリブル
            elif self.dribble_stage == 1:
                rospy.set_param("/robot_max_velocity", 0.5)
                pose_theta = math.atan2( (target_y - self.ctrld_robot.get_current_position()[1]) , (target_x - self.ctrld_robot.get_current_position()[0]) )
                area = 0.5
                if functions.distance_btw_two_points(self.ball_params.get_current_position(),
                                                     self.ctrld_robot.get_current_position()) \
                                                             > self.ctrld_robot.size_r + area: 

                    self.cmd.vel_surge = 0
                    self.cmd.vel_sway = 0
                    self.cmd.omega = 0
                    self.cmd.dribble_power = 0
                    self.status.robot_status = "None"
                    self.dribble_stage = 0
                    return

                if functions.distance_btw_two_points(self.ball_params.get_current_position(), [target_x, target_y]) < 0.3:
                    self.dribble_stage = 2

                self.cmd.dribble_power = self.dribble_power
                self.pid.pid_linear(target_x, target_y, pose_theta, ignore_penalty_area=ignore_penalty_area, avoid=False)

            #目標地付近になったらドリブラーを止めて引きずる(バックスピン防止)
            elif self.dribble_stage == 2:
                pose_theta = math.atan2( (target_y - self.ctrld_robot.get_current_position()[1]) , (target_x - self.ctrld_robot.get_current_position()[0]) )
                area = 0.5
                if functions.distance_btw_two_points(self.ball_params.get_current_position(),
                                                     self.ctrld_robot.get_current_position()) \
                        > self.ctrld_robot.size_r + area \
                        or functions.distance_btw_two_points(self.ball_params.get_current_position(), [target_x, target_y]) < 0.1:

                    self.cmd.vel_surge = 0
                    self.cmd.vel_sway = 0
                    self.cmd.omega = 0
                    self.cmd.dribble_power = 0
                    self.status.robot_status = "None"
                    self.dribble_stage = 0
                    return

                self.cmd.dribble_power = 0
                self.pid.pid_linear(target_x, target_y, pose_theta, ignore_penalty_area=ignore_penalty_area, avoid=False)
