#!/usr/bin/env python3
import rospy
import actionlib
from pal_presentation_msgs.msg import *


if __name__ == "__main__":

    client = actionlib.SimpleActionClient("/pal_play_presentation", pal_presentation_msgs.msg.PlayPresentationAction)
    client.wait_for_server()

    # check the pal_presentation_msgs/PlayPresentationGoal message
    # definition for the possible goal parameters
    goal = pal_presentation_msgs.msg.PlayPresentationGoal("joondalup")

    client.send_goal(goal)
    client.wait_for_result()

    rospy.loginfo("Action returned: %s" % client.get_result())