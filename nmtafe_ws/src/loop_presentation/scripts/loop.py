#!/usr/bin/env python3
import rospy
import actionlib
from pal_presentation_msgs.msg import *


if __name__ == "__main__":
    rospy.init_node('NMTAFE_loop_presentation')

    client = actionlib.SimpleActionClient("/pal_play_presentation_from_name", pal_presentation_msgs.msg.PlayPresentationFromNameAction)
    client.wait_for_server()

    # check the pal_presentation_msgs/PlayPresentationGoal message
    # definition for the possible goal parameters
    goal = pal_presentation_msgs.msg.PlayPresentationFromNameActionGoal()
    goal.goal.presentation_name = "joondalup"
    while True:
        client.send_goal(goal.goal)
        client.wait_for_result()
        rospy.loginfo("Action returned: %s" % client.get_result())