#!/usr/bin/env python
# -*- coding: utf-8 -*-

from aisaac.msg import Status
import rospy
import config
from strategy import StrategyBase

try:
    from typing import Dict, List
except:
    print("Module: typing (for better completion) not found. You can ignore this.")


class WorldModelStatusPublisher(object):
    def __init__(self, team_color, robot_ids=range(config.NUM_FRIEND_ROBOT)):
        # type: (str, objects.Objects, List[int]) -> None
        self._team_color = team_color
        self._robot_ids = robot_ids
        self._status_publishers = {}  # type: Dict[int, rospy.Publisher]

        for i in self._robot_ids:
            publisher = rospy.Publisher(
                "/" + self._team_color + "/robot_"+str(i)+"/status",
                Status,
                queue_size=10)
            self._status_publishers[i] = publisher

    def publish_all(self, strat):
        # type: (StrategyBase) -> None
        all_robot_status = strat.get_all_robot_status()

        for robot_id in all_robot_status.keys():
            self._status_publishers[robot_id].publish(
                all_robot_status[robot_id])
