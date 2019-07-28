#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from aisaac.msg import Status
import config


class StatusPublisher(object):
    """
    Methods
    ----------
    publish_all(strategy)
        publish strategy to all robots in the strategy
    """

    def __init__(self, team_color, robot_ids=range(config.NUM_FRIEND_ROBOT)):
        self._team_color = team_color
        self._robot_ids = robot_ids
        self._status_publishers = {}

        for i in self._robot_ids:
            publisher = rospy.Publisher(
                "/" + self._team_color + "/robot_"+str(i)+"/status",
                Status,
                queue_size=10)
            self._status_publishers[i] = publisher

    def publish_all(self, strategy):
        """
        Parameters
        ----------
        strategy : strategy_calcurtor.StrategyBase

        """

        all_robot_status = strategy.get_all_robot_status()

        for robot_id in all_robot_status.keys():
            self._status_publishers[robot_id].publish(
                all_robot_status[robot_id])
