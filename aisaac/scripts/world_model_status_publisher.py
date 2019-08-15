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
    def __init__(self, team_color, objects):
        # type: (str, objects.Objects, List[int]) -> None
        self._team_color = team_color
        self._objects = objects # type: objects.Objects
        self._status_publishers = {}  # type: Dict[int, rospy.Publisher]

        for i in range(self._objects.robot_total):
            publisher = rospy.Publisher(
                "/" + self._team_color + "/robot_"+str(i)+"/status",
                Status,
                queue_size=10)
            self._status_publishers[int(i)] = publisher

    def publish_all(self, strat):
        # type: (StrategyBase) -> None
        all_robot_status = strat.get_all_robot_status()

        for robot_id in all_robot_status.keys():
            if robot_id in self._objects.get_robot_ids():
                self._status_publishers[int(robot_id)].publish(all_robot_status[int(robot_id)])

    def set_objects(self, objects):
        self._objects = objects
