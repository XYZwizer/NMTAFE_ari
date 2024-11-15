#!/usr/bin/env python3
# -*- coding: utf-8 -*-
print("importing lib")
import rospy
import time

from actionlib import SimpleActionClient
from hri_msgs.msg import LiveSpeech
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from std_msgs.msg import String

from chat import ChatModel

print("done importing lib")



# The following demo subscribes to speech-to-text output and triggers TTS
# based on response

class ASR_llama_chat_bot(object):
    def __init__(self):
    	self.chatModel = chat.ChatModel()

        self.asr_sub = rospy.Subscriber(
            '/humans/voices/anonymous_speaker/speech',
            LiveSpeech,
            self.asr_result)

        self.tts_client = SimpleActionClient("/tts", TtsAction)
        self.tts_client.wait_for_server()

        self.language = "en_US"
        rospy.loginfo("ASR_llama_chat_bot ready")

    def asr_result(self, msg):

        # the LiveSpeech message has two main field: incremental and final.
        # 'incremental' is updated has soon as a word is recognized, and
        # will change while the sentence recognition progresses.
        # 'final' is only set at the end, when a full sentence is
        # recognized.
        sentence = msg.final
	if sentence = '':
		return
		
	self.tts_output(self.chatModel.Message(sentence))
	
        rospy.loginfo("Understood sentence: " + sentence)
	

    def tts_output(self, answer):

        self.tts_client.cancel_goal()
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = str(answer)
        self.tts_client.send_goal_and_wait(goal)

if __name__ == "__main__":
    rospy.init_node("asr_tutorial")
    node = ASRDemo()
    rospy.spin()