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

def toTwist(direc : tuple, boost : bool = False) -> Twist:
	twist_msg = Twist()
	
	forwardSpeed = 1 if boost else 0.3
	turnSpeed = 1 if boost else 0.6
	forwardValue = direc[1]
	turnValue = direc[0]
	if forwardValue < 0:
		turnValue = -turnValue
	
	twist_msg.linear.x = forwardValue * forwardSpeed
	twist_msg.linear.y = 0
	twist_msg.linear.z = 0
	twist_msg.angular.x = 0
	twist_msg.angular.y = 0
	twist_msg.angular.z = turnValue * turnSpeed
	return twist_msg

class ari_mover:
	def __init__(self):
		rospy.init_node('ari_controller_teli_NMTAFE')
		self.publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
		wait_for_subscribers(self.publisher)
		self.gamePad = Controller()
	def do(self):
		# disable this for release
		#if self.gamePad.west == 1:
		#	exit(0)   
		if self.gamePad.north == 1:
			self.gamePad.left_joy.setCurrentAsDeadZone()
		#print(f"{self.gamePad.left_joy.direction}", end="                        \r")
		self.publisher.publish(toTwist(self.gamePad.left_joy.direction, self.gamePad.lb))
