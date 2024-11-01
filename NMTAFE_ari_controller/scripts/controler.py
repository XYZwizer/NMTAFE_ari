from inputs import get_gamepad

import threading


class joyStick:

	defult_max = 32768

	y : int
	x : int
	def __init__(self) -> None:
		self.x = 0
		self.y = 0

		self.x_max = self.defult_max
		self.y_max = self.defult_max

		self.x_dead_zone = 0
		self.y_dead_zone = 0

	def setCurrentAsDeadZone(self):
		self.x_dead_zone = abs(self.x) + 2000
		self.y_dead_zone = abs(self.y) + 2000

	@property
	def direction(self):
		x = self.x/self.x_max if abs(self.x) > self.x_dead_zone else 0
		y = self.y/self.y_max if abs(self.y) > self.y_dead_zone else 0

		return (x,y)

class Controller:
	left_joy : joyStick

	right_joy : joyStick

	north : int
	east : int
	south : int
	west : int

	def __init__(self, background_polling=False) -> None:
		self.left_joy = joyStick()
		self.right_joy = joyStick()

		self.north = 0
		self.east = 0
		self.south = 0
		self.west = 0
		if background_polling:
			#print("starting polling thred")
			x = threading.Thread(target= self.__class__.background_poll, args=(self,))
			x.start()

	def background_poll(self):
		#print("thred started")
		while True:
			#print("polliugn")
			self.poll()
	def poll(self) -> None:
		events = get_gamepad()
		#print(events)
		for event in events:
			#match event.code:
			#	case "ABS_Y":
			#		self.left_joy.y = event.state
			#	case "ABS_X":
			#		self.left_joy.x = event.state
			#	case "ABS_RY":
			#		self.right_joy.y = event.state
			#	case "ABS_RX":
			#		self.right_joy.x = event.state
			if event.code == "ABS_Y":
				self.left_joy.y = event.state
			elif event.code == "ABS_X":
				self.left_joy.x = event.state
			elif event.code == "ABS_RY":
				self.right_joy.y = event.state
			elif event.code == "ABS_RX":
				self.right_joy.x = event.state
			elif event.code == "BTN_WEST":
				self.west = event.state
			elif event.code == "BTN_EAST":
				self.east = event.state
			elif event.code == "BTN_SOUTH":
				self.south = event.state
			elif event.code == "BTN_NORTH":
				self.north = event.state
