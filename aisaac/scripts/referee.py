# !/usr/bin/env  python
# coding:utf-8

from world_state import WorldState
import rospy
from consai_msgs.msg import RefereeTeamInfo
from std_msgs.msg import Int8

class Referee:
    def __init__(self, objects):
        self.team_color = objects.team_color
        self.world_state = WorldState(objects)

        """refereeから受信する情報"""
        self._command = None
        self._stage = None
        self._teaminfo = None

        """---Refereeから司令をもらうsubscriberの起動--"""
        rospy.Subscriber("/" + self.team_color + "/refbox/command", Int8, self._command_callback)
        rospy.Subscriber("/" + self.team_color + "/refbox/stage", Int8, self._stage_callback)
        rospy.Subscriber("/" + self.team_color + "/refbox/blue_info", RefereeTeamInfo, self._teaminfo_callback)

    """---Refereeからommandをもらう---"""
    def _command_callback(self, msg):
        self._command = str(msg)

    """---Refereeから現在のstageをもらう---"""
    def _stage_callback(self, msg):
        self._stage = str(msg)

    """---Refereeから現在のteaminfoをもらう---"""
    def _teaminfo_callback(self, msg):
        self._teaminfo = str(msg)

    def get_referee_msg(self):
        referee_msg = {
            'command': self._command,
            'stage': self._stage,
            'teaminfo': self._teaminfo
        }
        return referee_msg


    """---Refereeからの指示にしたがって行動指針を決定する---"""
    def get_referee_branch(self):
        """
        branch = 0 -> STOP
        branch = 1 -> far from ball
        branch = 2 -> normal
        branch = 3 -> kickoff
        """

        referee_branch = "STOP"

        if self._command == "data: 0":
            # Robots must keep 50 cm from the ball.
            referee_branch = "HALT"
        elif self._command == "data: 1":
            # A prepared kickoff or penalty may now be taken.
            referee_branch = "STOP"
        elif self._command == "data: 2":
            # The ball is dropped and free for either team.
            referee_branch = "NORMAL_START"

        elif self._command == "data: 3":
            # FORCE_START = 3
		    #The yellow team may move into kickoff position.
            referee_branch = "NORMAL_START"
        elif self._command == "data: 4":
            # PREPARE_KICKOFF_YELLOW = 4;
		    #The blue team may move into kickoff position.
            referee_branch = "KICKOFF"
        elif self._command == "data: 5":
            # PREPARE_KICKOFF_BLUE = 5;
		    # The yellow team may move into penalty position.
            referee_branch = "DEFENCE"
        elif self._command == "data: 6":
            # PREPARE_PENALTY_YELLOW = 6;
		    # The blue team may move into penalty position.
            referee_branch = "HALT"
        elif self._command == "data: 7":
            # PREPARE_PENALTY_BLUE = 7;
		    # The yellow team may take a direct free kick.
            referee_branch = "HALT"
        elif self._command == "data: 8":
            # DIRECT_FREE_YELLOW = 8;
		    # The blue team may take a direct free kick.
            referee_branch = "DIRECT_FREE_YELLOW"
        elif self._command == "data: 9":
            #DIRECT_FREE_BLUE = 9;
		    # The yellow team may take an indirect free kick.
            referee_branch = "DIRECT_FREE_BLUE"
        elif self._command == "data: 10":
            #INDIRECT_FREE_YELLOW = 10;
		    # The blue team may take an indirect free kick.
            referee_branch = "INDIRECT_FREE_YELLOW"
        elif self._command == "data: 11":
            # INDIRECT_FREE_BLUE = 11;
		    # The yellow team is currently in a timeout.
            referee_branch = "INDIRECT_FREE_BLUE"
        elif self._command == "data: 12":
            # TIMEOUT_YELLOW = 12;
		    # The blue team is currently in a timeout.
            referee_branch = "HALT"
        elif self._command == "data: 13":
            # TIMEOUT_BLUE = 13;
		    # The yellow team just scored a goal.
		    # For information only.
		    # For rules compliance, teams must treat as STOP.
            referee_branch = "HALT"
        elif self._command == "data: 14":
            # GOAL_YELLOW = 14;
		    # The blue team just scored a goal.
            referee_branch = "STOP"
        elif self._command == "data: 15":
            # GOAL_BLUE = 15;
            # Equivalent to STOP, but the yellow team must pick up the ball and
		    # drop it in the Designated Position.
            referee_branch = "STOP"
        elif self._command == "data: 16":
            #BALL_PLACEMENT_YELLOW = 16;
	        #Equivalent to STOP, but the blue team must pick up the ball and drop
		    # it in the Designated Position.
            referee_branch = "STOP"
        elif self._command == "data: 17":
            #BALL_PLACEMENT_BLUE = 17;
            referee_branch = "STOP"

        if self.world_state.color == "Blue":
            if self._command == "data: 4":
                referee_branch = "DEFENCE"
            elif self._command == "data: 5":
                referee_branch = "KICKOFF"

        return referee_branch
