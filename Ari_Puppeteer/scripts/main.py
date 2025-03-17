#!/usr/bin/env python3
import time
from geometry_msgs.msg import Twist
import rospy
from dualsense_controller import DualSenseController

class Puppeteer:
	def __init__(self):
		self.gamePad = GamepadInterface()
		self.movementInterface = RosMovementInterface()
		self.locked = False
		self.noController = True

	def Tick(self):
		if(self.gamePad.controller is None): #controller disconnected or not connected
			self.locked = True
			self.noController = True
		elif(self.noController): #controller reconnected
			self.noController = False
			self.gamePad.SetColor(255, 0, 0) #we are locked out

		if(self.gamePad.L1 and self.gamePad.R1 and not self.locked): #Killswitch
			self.locked = True
			self.gamePad.SetColor(255, 0, 0) #red signifies killswitch

		if(not self.locked):
			movement = self.gamePad.LS
			if(movement[1] < 0): #invert steering
				movement[0] = -movement[0]

			movementMagnitude = (movement[0] ** 2 + movement[1] ** 2) ** 0.5
			#scale between 0 and 255
			movementMagnitude = int(movementMagnitude * 255)
			movementMagnitude = min(255, movementMagnitude)
			movementMagnitude /= 4 #scale down to 0-64

			#add haptic feedback when in motion (so people dont forget they are piloting a robot)
			self.gamePad.SetRumble(movementMagnitude, movementMagnitude) 

			print(f"x: {movement[0]}, y: {movement[1]}")
			self.movementInterface.SetMovement(movement, 1)
		else:
			self.movementInterface.SetMovement((0, 0), 0)
			if(self.gamePad.Mic): #unlock
				self.locked = False
				self.gamePad.SetColor(10, 240, 10)

class GamepadInterface:
	def __init__(self):
		self.controller = None
		self.STICK_DEADZONE = 0.15
		self.LS = (0, 0) #left stick
		self.RS = (0, 0) #right stick
		self.L2 = 0 #left trigger
		self.R2 = 0 #right trigger
		self.L1 = False #left bumper
		self.R1 = False #right bumper
		self.Mic = False #mic button

		if(self.TryConnect()):
			print("Connected to Dualsense controller")
		else:
			print("Could not connect to Dualsense controller")
			#try to connect again in 3 seconds
			self.AttemptConnection()

	def AttemptConnection(self):
		connected = False
		while not connected:
			time.sleep(3)
			connected = self.TryConnect()
			if connected:
				print("Connected to Dualsense controller")
				self.SetColor(255, 255, 255) #white signifies connection
			else:
				print(".", end = "")

	def TryConnect(self):
		devices = DualSenseController.enumerate_devices()
		if len(devices) < 1:
			return False
		else:
			self.controller = DualSenseController()
			self.controller.left_stick_x.on_change(self.OnLeftStickX)
			self.controller.left_stick_y.on_change(self.OnLeftStickY)
			self.controller.L1.on_change(self.OnL1)
			self.controller.R1.on_change(self.OnR1)
			self.controller.Mute.on_change(self.OnMic)
			self.controller.on_error(self.OnError)

			self.controller.activate()
			return True
	
	def	OnError(self, error):
		print(f"Error: {error}")
		self.controller.deactivate()
		self.controller = None #failsafe (dont keep this, ideal to change it when we know possible errors)
	def OnLeftStickX(self, x):
		if abs(x) < self.STICK_DEADZONE:
			x = 0
		self.LS = (x, self.LS[1])
	def OnLeftStickY(self, y):
		if abs(y) < self.STICK_DEADZONE:
			y = 0
		self.LS = (self.LS[0], y)
	def OnL1(self, pressed):
		self.L1 = pressed
	def OnR1(self, pressed):
		self.R1 = pressed
	def OnMic(self, pressed):
		self.Mic = pressed
	
	def SetColor(self, r, g, b):
		if self.controller is not None:
			self.controller.lightbar.set_color(r, g, b)

	def SetRumble(self, left, right):
		if self.controller is not None:
			self.controller.left_rumble.set(left)
			self.controller.right_rumble.set(right)


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