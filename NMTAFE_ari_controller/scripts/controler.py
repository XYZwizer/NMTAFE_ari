import inputs
import multiprocessing
import time

class joyStick_axis:
	def __init__(self):
		self.max = 255
		self.min = 0
		
		self.dead_zone = 0.03
		
		self.mp_real_value = multiprocessing.Value('f')
		
		self.real_value = (self.max+self.min)/2 #should give nutral for range so that we start at 0

	@property
	def real_value(self):
		return self.mp_real_value.value
	@real_value.setter
	def real_value(self,v):
		self.mp_real_value.value = v
	@property
	def norm(self):
		normed = -(-1.0 + (2.0/(self.max-self.min)) * (self.real_value - self.min))
		if abs(normed) < self.dead_zone:
			return 0.0
		return normed
	

class joyStick:
	x : int
	y : int
	def __init__(self) -> None:
		self.x = joyStick_axis()
		self.y = joyStick_axis()

	def setCurrentAsMax(self):
		self.x.max = self.x.real_value
		self.y.max = self.x.real_value
		
	def setCurrentAsMin(self):
		self.x.min = self.x.real_value
		self.y.min = self.x.real_value

	def setCurrentAsDeadZone(self):
		self.x.dead_zone = abs(self.x.norm) + 0.2
		self.y.dead_zone = abs(self.y.norm) + 0.2

	@property
	def direction(self):
		return (self.x.norm,self.y.norm)
			
class Controller:
	def __init__(self) -> None:
		Backend = _ControllerBackend()
		self.left_joy = joyStick()
		self.right_joy = joyStick()
		
		self.mp_north = multiprocessing.Value('i')
		self.mp_east = multiprocessing.Value('i')
		self.mp_south = multiprocessing.Value('i')
		self.mp_west = multiprocessing.Value('i')
		self.mp_lb = multiprocessing.Value('i')
		self.mp_rb = multiprocessing.Value('i')
		self.mp_lt = multiprocessing.Value('i')
		self.mp_rt = multiprocessing.Value('i')
		self.mp_back = multiprocessing.Value('i')
		self.mp_start = multiprocessing.Value('i')

		self.background_polling = multiprocessing.Process(target=Backend.do_polling, args=(self,))
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

	@property
	def lb(self):
		return self.mp_lb.value
	@lb.setter
	def lb(self,v):
		self.mp_lb.value = v

	@property
	def rb(self):	
		return self.mp_rb.value
	@rb.setter
	def rb(self,v):
		self.mp_rb.value = v

	@property
	def lt(self):
		return self.mp_lt.value
	@lt.setter
	def lt(self,v):
		self.mp_lt.value = v

	@property
	def rt(self):
		return self.mp_rt.value
	@rt.setter
	def rt(self,v):
		self.mp_rt.value = v

	@property
	def back(self):
		return self.mp_back.value
	@back.setter
	def back(self,v):
		self.mp_back.value = v

	@property
	def start(self):
		return self.mp_start.value
	@start.setter
	def start(self,v):
		self.mp_start.value = v

	def poll(self) -> None:
		events = inputs.get_gamepad()
		
		for event in events:
			if event.code == "ABS_Y":
				self.left_joy.y.real_value = event.state
			elif event.code == "ABS_X":
				self.left_joy.x.real_value = event.state
			elif event.code == "ABS_RY":
				self.right_joy.y.real_value = event.state
			elif event.code == "ABS_RX":
				self.right_joy.x.real_value = event.state
			elif event.code == "BTN_WEST":
				self.west = event.state
			elif event.code == "BTN_EAST":
				self.east = event.state
			elif event.code == "BTN_SOUTH":
				self.south = event.state
			elif event.code == "BTN_NORTH":
				self.north = event.state
			elif event.code == "BTN_TR1":
				self.rb = event.state
			elif event.code == "BTN_TL1":
				self.lb = event.state
			elif event.code == "BTN_TR2":
				self.rt = event.state
			elif event.code == "BTN_TL2":
				self.lt = event.state
			# elif event.code == "BTN_START":
			# 	self.start = event.state
			# elif event.code == "BTN_SELECT":
			# 	self.back = event.state

class _ControllerBackend:
	def wait_for_gamepad(self):
				try:
					inputs.devices = inputs.DeviceManager()
					while len(inputs.devices.gamepads) == 0:
						time.sleep(2)
						inputs.devices = inputs.DeviceManager()
					print("---gamepad found---", inputs.devices.gamepads)
				except:
					print("detecting disconceed gamepad pls WAIT",end="\r")
	
	def do_polling(self,controllor):
		while True:
			try:
				controllor.poll()
			except inputs.UnpluggedError:
				print("no game pad detected waiting for gamepad")
				self.wait_for_gamepad()
			except OSError as e:
				print("gamepad disconnected pls restart program to continue")
				self.wait_for_gamepad()
				