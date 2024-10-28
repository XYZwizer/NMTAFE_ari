from controler import Controller
import rospy

from geometry_msgs.msg import Twist

def wait_for_subscribers(publisher):
	i = 0
	while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
		if i == 4:
			print("Waiting for subscriber to connect to {}".format(publisher.name))
		rospy.sleep(0.5)
		i += 1
		i = i % 5
		if rospy.is_shutdown():
			raise Exception("Got shutdown request before subscribers connected")

def toTwist(direc : tuple):
	twist_msg = Twist()
	
	speed = 1.0
	
	twist_msg.linear.x = -direc[1] * speed
	twist_msg.linear.y = 0#direc[1] * speed
	twist_msg.linear.z = 0#direc[0] * speed
	twist_msg.angular.x = 0
	twist_msg.angular.y = 0
	twist_msg.angular.z = -direc[0] * speed
	return twist_msg

class ari_mover:
	def __init__(self):
		rospy.init_node('cont_teli')
		self.publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 1)
		wait_for_subscribers(self.publisher)
		self.gamePad = Controller(background_polling=True)
	def do(self):
		if self.gamePad.west == 1:
			exit(0)
		if self.gamePad.north == 1:
			self.gamePad.left_joy.setCurrentAsDeadZone()
		print(f"{self.gamePad.left_joy.direction}", end="                        \r")
		self.publisher.publish(toTwist(self.gamePad.left_joy.direction))
