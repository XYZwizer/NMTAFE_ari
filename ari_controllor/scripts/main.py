#!/usr/bin/env python3
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

if __name__ == "__main__":
	print("david")
	rospy.init_node('cont_teli')
	publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 1)
	wait_for_subscribers(publisher)
	c = Controller()
	while(1):
		c.poll()
		if c.west == 1:
			exit(0)
		if c.north == 1:
			c.left_joy.setCurrentAsDeadZone()
		print(f"{c.left_joy.direction}", end="                        \r")
		publisher.publish(toTwist(c.left_joy.direction))
		
