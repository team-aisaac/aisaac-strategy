#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslaunch
import subprocess
import time
import rospy
import sys

package = 'aisaac'
executable = 'robot.py'
rospy.set_param('is_test', True)
rospy.set_param('team_side', 'left')
rospy.set_param('friend_color', 'blue')
node = roslaunch.core.Node(package, executable, args='_robot_num:=0')


launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
process = launch.launch(node)

time.sleep(5)
if process.is_alive():
    process.stop()
else:
    process.stop()
    sys.exit(1)

