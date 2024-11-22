#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from hri_msgs.msg import LiveSpeech
from actionlib import SimpleActionClient
import asyncio
import time
import rospy
import sys
from remotellama2 import LlamaInterface

# The following demo subscribes to speech-to-text output and triggers TTS
# based on response

class ASR_llama_chat_bot(object):
    def __init__(self, ip):
        self.chatModel = LlamaInterface(ip)

        self.asr_sub = rospy.Subscriber(
            '/humans/voices/anonymous_speaker/speech',
            LiveSpeech,
            self.asr_result)

        self.tts_client = SimpleActionClient("/tts", TtsAction)
        self.tts_client.wait_for_server()

        self.language = "en_US"
        rospy.loginfo("ASR_llama_chat_bot ready")
        print("ASR_llama_chat_bot ready")
        self.processing = False

    def asr_result(self, msg):
        # the LiveSpeech message has two main field: incremental and final.
        # 'incremental' is updated has soon as a word is recognized, and
        # will change while the sentence recognition progresses.
        # 'final' is only set at the end, when a full sentence is
        # recognized.
        sentence = msg.final
        sentence_word_count = len(sentence.split())
        empty_sentence = sentence == ''
        sentence_word_count_less_than_4 = sentence_word_count < 4
        processing = self.processing
        robot_not_in_sentence = not "robot" in sentence.lower()
        if empty_sentence or sentence_word_count_less_than_4 or processing or robot_not_in_sentence:
            rospy.loginfo("Ignoring sentence: " + sentence + "empty_sentence: " + str(empty_sentence) + " sentence_word_count_less_than_4: " + str(sentence_word_count_less_than_4) + " processing: " + str(processing) + " robot_not_in_sentence: " + str(robot_not_in_sentence))
            return
        self.processing = True
        rospy.loginfo("Understood sentence: " + sentence)
        self.tts_output(self.chatModel.query(sentence))
        asyncio.run(self.clear_message_queue())

    async def clear_message_queue(self):
        #wait 1 second asynchronusly
        await asyncio.sleep(1)
        self.processing = False

    def tts_output(self, answer):
        self.tts_client.cancel_goal()
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = str(answer)
        self.tts_client.send_goal_and_wait(goal)


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    rospy.init_node("asr_llama_chat_bot")
    node = ASR_llama_chat_bot(args[1])
    rospy.spin()