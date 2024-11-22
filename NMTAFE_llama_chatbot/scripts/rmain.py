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
        systemprompt = """Please respond in under twenty (20) words.
You are Isabella a robot manufactured by pal robotics and operated by the city of JOON-da-lup and N.M. Tayfe.
your are here to assist with questions about JOON-da-lup and the N.M. Tayfe Campus.
you are a humanoid robot with 2 arms, a white and yellow body and a screen on your chest.
"""
        self.chatModel.set_system_prompt(systemprompt)
        self.asr_sub = rospy.Subscriber(
            '/humans/voices/anonymous_speaker/speech',
            LiveSpeech,
            self.asr_result)

        self.tts_client = SimpleActionClient("/tts", TtsAction)
        

        self.language = "en_US"
        rospy.loginfo("ASR_llama_chat_bot ready")
        #print("ASR_llama_chat_bot ready")
        self.processing = False
        self.tts_client.wait_for_server()

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
        skip = self.processing
        robot_not_in_sentence = not "isabella" in sentence.lower()
        if empty_sentence or sentence_word_count_less_than_4 or skip or robot_not_in_sentence:
            if not empty_sentence:
                rospy.loginfo("Ignoring sentence: " + sentence + " || under4: " + str(sentence_word_count_less_than_4) + " halted: " + str(skip) + " no robot: " + str(robot_not_in_sentence))
            return

        self.processing = True
        rospy.loginfo("\nUnderstood: " + sentence)
        response = self.chatModel.query(sentence)
        rospy.loginfo("Got Response")
        #if the response is over 20 words, reapply the system prompt
        bad_message = len(response.split()) > 25
        if bad_message:
            rospy.loginfo("\nResponse too long: " + response + "\n")
            self.chatModel.query("You have just been reset", reset="true")
        else:
            self.tts_output(response)
            rospy.loginfo("\nResponding: " + response + "\n")
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
        self.tts_client.send_goal(goal)


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    rospy.init_node("asr_llama_chat_bot")
    node = ASR_llama_chat_bot(args[1])
    rospy.spin()