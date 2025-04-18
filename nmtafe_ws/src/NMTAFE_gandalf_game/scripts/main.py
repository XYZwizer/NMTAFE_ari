from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from hri_msgs.msg import LiveSpeech
from hri_msgs.msg import LiveSpeech

from emoji_to_emote import do_emot_from_emoji
from am_looked_at import BodyOrientationListener

from enum import Enum

# class syntax
class game_state(Enum):
    awating_player = 0
    engaged_with_player = 1

class NMTAFE_gandalf_game_node:
	def __init__(self):
		rospy.init_node("NMTAFE_gandalf_game")
		self.tts_client = SimpleActionClient("/tts", TtsAction)
		self.asr_sub = rospy.Subscriber(
			'/humans/voices/anonymous_speaker/speech',
			LiveSpeech,
			self.on_speech)
        self.game_state = game_state.awating_player


	def on_speech(self,text):
        

	def tts_output(self, answer):
		self.tts_client.cancel_goal()
		goal = TtsGoal()
		goal.rawtext.lang_id = "en_US"
		goal.rawtext.text = str(answer)
		self.tts_client.send_goal_and_wait(goal)
	
if __name__ == '__main__':
	# execute only if run as the entry point into the program
	node = NMTAFE_gandalf_game_node()
	rospy.spin()