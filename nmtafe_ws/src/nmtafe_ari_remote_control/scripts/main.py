#!/usr/bin/env python3.10
import time
from dualsense_controller import DualSenseController
from geometry_msgs.msg import Twist
import rospy
import os

class Puppeteer:
	def __init__(self):
		self.gamePad = GamepadInterface()
		self.movementInterface = RosMovementInterface()
		self.locked = False
		self.controllerActive = False

	def Tick(self):
		if(self.gamePad.controller is None and self.controllerActive): #controller disconnected or not connected
			self.locked = True
			self.controllerActive = False
			print("Controller disconnected")

		elif(not self.controllerActive and self.gamePad.controller): #controller reconnected
			self.gamePad.SetColor(255, 0, 0) #we are locked out
			self.locked = True
			self.controllerActive = True

		if(self.gamePad.L1 and self.gamePad.R1 and not self.locked): #Killswitch
			self.locked = True
			self.gamePad.SetColor(255, 0, 0) #red signifies killswitch

		if(not self.locked):
			self.gamePad.Timeout += 1 #increment timeout
			if(self.gamePad.Timeout > 5): #timeout after 1 second
				self.locked = True
				self.gamePad.SetColor(255, 0, 0) #red signifies killswitch
				print("Controller timeout")
				return
			movement = self.gamePad.LS
			if(movement[1] < 0): #invert steering
				movement = (-movement[0], movement[1])

			movementMagnitude = (movement[0] ** 2 + movement[1] ** 2) ** 0.5
			if(movementMagnitude > 1):

				#scale between 0 and 0.5
				movementMagnitude *= 0.5
				if(movementMagnitude > 0.5):
					movementMagnitude = 0.5
				
				#Apply exponential scaling to make fine control easier and allow for higher speeds
				range = 128 #1024 for full range
				movementMagnitude = (range * (movementMagnitude ** 2)) - 1  # Square for exponential curve
				#print(f"Movement magnitude: {movementMagnitude}")
				leftSide = movementMagnitude
				rightSide = movementMagnitude
				#Scale reduction based on left stick X value
				turn_factor = abs(self.gamePad.LS[0])  # How much we're turning
				if self.gamePad.LS[0] > 0:  # Turning right
					leftSide *= (1.03 - turn_factor)  # Reduce left side proportionally
				else:  # Turning left
					rightSide *= (1 - turn_factor)  # Reduce right side proportionally

				#add haptic feedback when in motion (so people dont forget they are piloting a robot)
				self.gamePad.SetRumble(leftSide, rightSide)
			else:
				self.gamePad.SetRumble(0, 0)

			print(f"Dualsense x: {movement[0]}, y: {movement[1]}")
			self.movementInterface.SetMovement(movement, 0.5)
		else:
			self.movementInterface.SetMovement((0, 0), 0)
			#print(self.gamePad.Mic)
			if(self.gamePad.Mic): #unlock
				self.locked = False
				self.gamePad.SetColor(10, 240, 10)

class GamepadInterface:
	def __init__(self):
		self.controller = None
		self.LS = (0, 0) #left stick
		self.RS = (0, 0) #right stick
		self.L2 = 0 #left trigger
		self.R2 = 0 #right trigger
		self.L1 = False #left bumper
		self.R1 = False #right bumper
		self.Mic = False #mic button
		self.Timeout = 0 #timeout for connection attempts

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
				os.system("./pair_to_dedicated_dualsense.sh")
				print(".", end = "")

	def TryConnect(self):
		devices = DualSenseController.enumerate_devices()
		if len(devices) < 1:
			return False
		else:
			self.controller = DualSenseController(left_joystick_deadzone=0.08, right_joystick_deadzone=0.08)
			self.controller.left_stick_x.on_change(self.OnLeftStickX)
			self.controller.left_stick_y.on_change(self.OnLeftStickY)
			self.controller.btn_l1.on_change(self.OnL1)
			self.controller.btn_r1.on_change(self.OnR1)
			self.controller.btn_mute.on_change(self.OnMic)
			#self.controller.btn_ps.on_change(self.OnMic) #backup
			self.controller.gyroscope.on_change(self.OnGyro)
			self.controller.on_error(self.OnError)

			self.controller.activate()
			return True
	
	def	OnError(self, error):
		tController = self.controller
		self.controller = None #failsafe (dont keep this, ideal to change it when we know possible errors)
		print(f"Error: {error}")
		tController.deactivate()
	def OnLeftStickX(self, x):
		self.LS = (x, self.LS[1])
	def OnLeftStickY(self, y):
		print(y)
		self.LS = (self.LS[0], y)
	def OnL1(self, pressed):
		self.L1 = pressed
	def OnR1(self, pressed):
		self.R1 = pressed
	def OnMic(self, pressed):
		self.Mic = pressed
		print(f"Mic: {pressed}")

	def OnGyro(self, gyro):
		print(f"Gyro: {gyro}")
		self.Timeout = 0 #reset	 timeout
	
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
