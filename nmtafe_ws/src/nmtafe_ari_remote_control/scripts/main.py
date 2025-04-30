#!/usr/bin/env python3
import Gamepad
import time
from geometry_msgs.msg import Twist
import rospy

class Puppeteer:
	def __init__(self):
		self.gamePad = GamepadInterface()
		self.movementInterface = RosMovementInterface()
		self.locked = False

	def Tick(self):
		(x, y) = self.gamePad.GetInput('LEFT-X', "axis"), self.gamePad.GetInput('LEFT-Y', "axis")
		L1 = self.gamePad.GetInput('L1', "pressed")
		R1 = self.gamePad.GetInput('R1', "pressed")
		PS = self.gamePad.GetInput('PS', "pressed")

		if(L1 and R1): #Killswitch
			self.locked = True

		if(not self.locked):
			if(x is not None and y is not None):
				#deadzone check
				if(abs(x) < 0.2):
					x = 0
				if(abs(y) < 0.2):
					y = 0
				if(y < 0): #invert steering
					x = -x
				print(f"x: {x}, y: {y}")
				self.movementInterface.SetMovement((x, y), 1)
			else:
				self.movementInterface.SetMovement((0, 0), 0)
				self.locked = True
		else:
			self.movementInterface.SetMovement((0, 0), 0)
			print("Locked")
			if(PS is not None and PS):
				self.locked = False
				print("Unlocked")

class GamepadInterface:
	def __init__(self):
		self.gamePadType = Gamepad.PS4
		self.gamePad = None

	def GetInput(self, inputName: str, inputType: str):
		if(self.gamePad is not None and self.gamePad.isConnected()):
			if(inputType == "pressed"):
				return self.gamePad.isPressed(inputName)
			elif(inputType == "axis"):
				return self.gamePad.axis(inputName)
		else:
			if(self.gamePad is None):
				print("Gamepad is not connected")
				if(Gamepad.available(1)):
					print("Gamepad is available")
					self.gamePad = self.gamePadType(1)
					self.gamePad.startBackgroundUpdates()
			elif(not self.gamePad.isConnected()):
				print("Gamepad is not connected")
				self.gamePad.disconnect()
				self.gamePad = None
			else:
				print("Gamepad is not connected")
			return None
		


class RosMovementInterface:
	def __init__(self):
		self.hasPublisher = False
		rospy.init_node('ari_controller_teli_NMTAFE2')
		self.publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
		self._wait_for_subscribers(self.publisher)

	def _wait_for_subscribers(self, publisher):
		i = 0
		while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
			if i == 4:
				print("Waiting for subscriber to connect to {}".format(publisher.name))
			rospy.sleep(0.5)
			i += 1
			i = i % 5
			if rospy.is_shutdown():
				raise Exception("Got shutdown request before subscribers connected")
		self.hasPublisher = True

	def _toTwist(self, direc : tuple, speed : float) -> Twist:
		twist_msg = Twist()
		twist_msg.linear.x = -direc[1] * speed
		twist_msg.linear.y = 0
		twist_msg.linear.z = 0
		twist_msg.angular.x = 0
		twist_msg.angular.y = 0
		twist_msg.angular.z = -direc[0] * speed
		return twist_msg
	
	def SetMovement(self, movement, speed):
		if self.hasPublisher:
			self.publisher.publish(self._toTwist(movement, speed))
		else:
			print("No publisher available")
	
	

if __name__ == "__main__":
	ari = Puppeteer()
	while not rospy.is_shutdown():
		ari.Tick()
		time.sleep(0.1)