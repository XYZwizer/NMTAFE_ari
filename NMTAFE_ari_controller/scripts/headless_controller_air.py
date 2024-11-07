#!/usr/bin/env python3
from controler import Controller
import rospy
from geometry_msgs.msg import Twist

class ari_mover2:
	def __init__(self):
		rospy.init_node('ari_controller_teli_NMTAFE')
		self.publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
		self._wait_for_subscribers(self.publisher)
		self.gamePad = Controller(background_polling=True)      
	
	def mainloop(self):
		#check if our gamepad is connected
		if self.gamePad is None:
			self.gamePad = Controller(background_polling=True)
			self.gamePad.wait_for_gamepad()
		else:
			if self.gamePad.west == 1:
				exit(0)
			if self.gamePad.north == 1:
				self.gamePad.left_joy.setCurrentAsDeadZone()
			print(f"{self.gamePad.left_joy.direction}", end="                        \r")
			self.publisher.publish(self._toTwist(self.gamePad.left_joy.direction))

	def _wait_for_subscribers(publisher):
		i = 0
		while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
			if i == 4:
				print("Waiting for subscriber to connect to {}".format(publisher.name))
			rospy.sleep(0.5)
			i += 1
			i = i % 5
			if rospy.is_shutdown():
				raise Exception("Got shutdown request before subscribers connected")

	def _toTwist(direc : tuple):
		twist_msg = Twist()
		
		speed = 1.0
		
		twist_msg.linear.x = -direc[1] * speed
		twist_msg.linear.y = 0#direc[1] * speed
		twist_msg.linear.z = 0#direc[0] * speed
		twist_msg.angular.x = 0
		twist_msg.angular.y = 0
		twist_msg.angular.z = -direc[0] * speed
		return twist_msg

if __name__ == "__main__":
	ari = ari_mover2()
	while(1):
		ari._mainloop()
		