import inputs

import multiprocessing
import time

class joyStick:

	posibel_max = [127,32768]
	
	defult_max = posibel_max[-1]

	y : int
	x : int
	def __init__(self) -> None:
		self.mp_x = multiprocessing.Value('f')
		self.mp_y = multiprocessing.Value('f')

		self.x_max = self.defult_max
		self.y_max = self.defult_max

		self.x_dead_zone = 0
		self.y_dead_zone = 0
		
	def findMax(self):
		new_max = min(self.posibel_max, key=lambda i:abs(i-self.x))
		self.x_max = new_max
		self.y_max = new_max

	def setCurrentAsDeadZone(self):
		self.x_dead_zone = abs(self.p_x) + 0.2
		self.y_dead_zone = abs(self.p_y) + 0.2
		print(self.x_dead_zone)

	@property
	def direction(self):
		x = self.p_x if abs(self.p_x) > self.x_dead_zone else 0.0
		y = self.p_y if abs(self.p_y) > self.y_dead_zone else 0.0

		return (x,y)
		
	@property
	def p_x(self):
		return self.x/self.x_max
		
	@property
	def p_y(self):
		return self.x/self.x_max
		
	@property
	def x(self):
		return self.mp_x.value
	@x.setter
	def x(self,v):
		self.mp_x.value = v
		
	@property
	def y(self):
		return self.mp_y.value
	@y.setter
	def y(self,v):
		self.mp_y.value = v
		
	
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

		#self.north = 0
		#self.east = 0
		#self.south = 0
		#self.west = 0
		
		self.mp_north = multiprocessing.Value('i')
		self.mp_east = multiprocessing.Value('i')
		self.mp_south = multiprocessing.Value('i')
		self.mp_west = multiprocessing.Value('i')
		
		self.background_polling = multiprocessing.Process(target=self.do_polling, args=(self,))
		self.background_polling.start()
		
	@property 
	def north(self):
		return self.mp_north.value
	@north.setter
	def north(self,v):
		self.mp_north.value = v
		
	@property 
	def east(self):
		return self.mp_east.value
	@east.setter
	def east(self,v):
		self.mp_east.value = v
	
	@property 
	def south(self):
		return self.mp_south.value
	@south.setter
	def south(self,v):
		self.mp_south.value = v
		
	@property 
	def west(self):
		return self.mp_west.value
	@west.setter
	def west(self,v):
		self.mp_west.value = v
		
	def wait_for_gamepad(self):
				inputs.devices._post_init()	
				print("am=======================",inputs.devices)
				while len(inputs.devices.gamepads) == 0:
					time.sleep(2)
					inputs.devices._post_init()
				print("---gamepad found---")
		
	def do_polling(self,controllor):
		while True:
			try:
				controllor.poll()
			except inputs.UnpluggedError:
				print("no game pad detected waiting for gamepad")
				self.wait_for_gamepad()
			except OSError as e:
				print("gamepad disconnected pls restart program to continue")
				
	def poll(self) -> None:
		events = inputs.get_gamepad()
		
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
