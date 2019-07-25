import rospy
from consai_msgs.msg import RefereeTeamInfo

class WorldModelRefereeSubsriber(object):
    def __init__(self, team_color):

        """---Refereeから司令をもらうsubscriberの起動--"""
        """ rospy.Subscriber("refbox/command", Int8, self.referee.command_callback)
        rospy.Subscriber("refbox/stage", Int8, self.referee.stage_callback)
        rospy.Subscriber("refbox/blue_info", RefereeTeamInfo, self.referee.teaminfo_callback) """

        rospy.Subscriber("/" + self.team_color + "/refbox/command", Int8, self.referee.command_callback)
        rospy.Subscriber("/" + self.team_color + "/refbox/stage", Int8, self.referee.stage_callback)
        rospy.Subscriber("/" + self.team_color + "/refbox/blue_info", RefereeTeamInfo, self.referee.teaminfo_callback)
