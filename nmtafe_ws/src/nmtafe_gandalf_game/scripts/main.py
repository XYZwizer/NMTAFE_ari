#!/usr/bin/env python
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from hri_msgs.msg import LiveSpeech
from actionlib import SimpleActionClient

from emoji_to_emote import do_emot_from_emoji
from am_looked_at import BodyOrientationListener

from pal_web_msgs.msg import WebGoTo

import rospy

from gandalf_game_api import gandalf_game

# class syntax
class game_state:
	awating_player = 0
	engaged_with_player = 1
	playing = 2

class NMTAFE_gandalf_game_node:
	def __init__(self):
		rospy.init_node("NMTAFE_gandalf_game")
		self.tts_client = SimpleActionClient("/tts", TtsAction)
		self.tts_client.wait_for_server()

		self.asr_sub = rospy.Subscriber(
			'/humans/voices/anonymous_speaker/speech',
			LiveSpeech,
			self.on_speech)
		
		self.current_game = gandalf_game()

		self.pas_sub = rospy.Subscriber("/NMTAFE_gandalf_game/password_attempt", String, self.on_password_attempt)
		#self.force_game_state_sub = rospy.Subscriber("/NMTAFE_gandalf_game/password_attempt",std_msgs.msg.bool, self.on_force_game_state)

		self.web_pub = rospy.Publisher('/web/go_to',  WebGoTo, queue_size=10) 
		
		self.game_state = game_state.awating_player
		self.set_game_state(game_state.playing)
	
	def set_page(self,page_name):
		new_page = WebGoTo(type=WebGoTo.TOUCH_PAGE,value=page_name)
		self.web_pub.publish(new_page)

	def set_game_state(self,new_state):
		if new_state == game_state.playing:
			print("now playing new game")
			self.current_game = gandalf_game()
			self.set_page("gandalf_game_password_input")
			self.output_tts(self.current_game.get_description())
		print(self.game_state,new_state)

	def on_speech(self,text):
		if not text.final: return
		
		if "yes" in text.final:
			self.set_game_state(game_state.playing)

	def on_password_attempt(self,password):
		print("doing g game stuff:", password)

	def on_force_game_state(self, game_on):
		if game_on:
			self.set_game_state(game_state.playing)
		else:
			self.set_game_state(game_state.awating_player) 


	def output_tts(self, answer):
		self.tts_client.cancel_goal()
		goal = TtsGoal()
		goal.rawtext.lang_id = "en_GB"
		goal.rawtext.text = str(answer)
		print(goal.rawtext.text,goal.rawtext.lang_id)
		self.tts_client.send_goal_and_wait(goal)
	
if __name__ == '__main__':
	# execute only if run as the entry point into the program
	node = NMTAFE_gandalf_game_node()
	#foobar = BodyOrientationListener()
	#foobar.run()
	rospy.spin()
