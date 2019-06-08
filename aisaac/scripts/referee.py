# !/usr/bin/env  python
# coding:utf-8


class Referee:
    def __init__(self, world_state):
        self.world_state = world_state
        """---refereeからの指示をもとにどのループを回すかの指標---"""
        self.referee_branch = None

        """refereeから受信する情報"""
        self.command = None
        self.stage = None
        self.teaminfo = None

    """---Refereeから現在のcommandをもらう---"""
    def command_callback(self, msg):
        self.command = str(msg)

    """---Refereeから現在のstageをもらう---"""
    def stage_callback(self, msg):
        self.stage = str(msg)

    """---Refereeから現在のteaminfoをもらう---"""
    def teaminfo_callback(self, msg):
        self.teaminfo = str(msg)

    """---Refereeからの指示にしたがって行動指針を決定する---"""
    def referee_branch_decision(self):
        """
        branch = 0 -> STOP
        branch = 1 -> far from ball
        branch = 2 -> normal
        branch = 3 -> kickoff
        """
        #   print(self.command)
        if self.command == "data: 0":
            # Robots must keep 50 cm from the ball.
            self.referee_branch = "HALT"
        elif self.command == "data: 1":
            # A prepared kickoff or penalty may now be taken.
            self.referee_branch = "STOP"
        elif self.command == "data: 2":
            # The ball is dropped and free for either team.
            self.referee_branch = "NORMAL_START"

        elif self.command == "data: 3":
            # FORCE_START = 3
		    #The yellow team may move into kickoff position.
            self.referee_branch = "NORMAL_START"
        elif self.command == "data: 4":
            # PREPARE_KICKOFF_YELLOW = 4;
		    #The blue team may move into kickoff position.
            self.referee_branch = "KICKOFF"
        elif self.command == "data: 5":
            # PREPARE_KICKOFF_BLUE = 5;
		    # The yellow team may move into penalty position.
            self.referee_branch = "DEFENCE"
        elif self.command == "data: 6":
            # PREPARE_PENALTY_YELLOW = 6;
		    # The blue team may move into penalty position.
            self.referee_branch = "HALT"
        elif self.command == "data: 7":
            # PREPARE_PENALTY_BLUE = 7;
		    # The yellow team may take a direct free kick.
            self.referee_branch = "HALT"
        elif self.command == "data: 8":
            # DIRECT_FREE_YELLOW = 8;
		    # The blue team may take a direct free kick.
            self.referee_branch = "HALT"
        elif self.command == "data: 9":
            #DIRECT_FREE_BLUE = 9;
		    # The yellow team may take an indirect free kick.
            self.referee_branch = "HALT"
        elif self.command == "data: 10":
            #INDIRECT_FREE_YELLOW = 10;
		    # The blue team may take an indirect free kick.
            self.referee_branch = "HALT"
        elif self.command == "data: 11":
            # INDIRECT_FREE_BLUE = 11;
		    # The yellow team is currently in a timeout.
            self.referee_branch = "HALT"
        elif self.command == "data: 12":
            # TIMEOUT_YELLOW = 12;
		    # The blue team is currently in a timeout.
            self.referee_branch = "HALT"
        elif self.command == "data: 13":
            # TIMEOUT_BLUE = 13;
		    # The yellow team just scored a goal.
		    # For information only.
		    # For rules compliance, teams must treat as STOP.
            self.referee_branch = "HALT"
        elif self.command == "data: 14":
            # GOAL_YELLOW = 14;
		    # The blue team just scored a goal.
            self.referee_branch = "STOP"
        elif self.command == "data: 15":
            # GOAL_BLUE = 15;
            # Equivalent to STOP, but the yellow team must pick up the ball and
		    # drop it in the Designated Position.
            self.referee_branch = "STOP"
        elif self.command == "data: 16":
            #BALL_PLACEMENT_YELLOW = 16;
	        #Equivalent to STOP, but the blue team must pick up the ball and drop
		    # it in the Designated Position.
            self.referee_branch = "STOP"
        elif self.command == "data: 17":
            #BALL_PLACEMENT_BLUE = 17;
            self.referee_branch = "STOP"

        if self.world_state.color == "Blue":
            if self.command == "data: 4":
                self.referee_branch = "DEFENCE"
            elif self.command == "data: 5":
                self.referee_branch = "KICKOFF"
