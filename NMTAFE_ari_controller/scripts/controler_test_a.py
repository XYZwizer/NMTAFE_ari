from inputs import get_gamepad
import time
import asyncio


#def my_do_iter(self):
	#self._character_file = io.open(self._character_device_path, 'rb')
	#self._character_device.read(10000)
	#read_size = self._get_total_read_size()
	#data = self._get_data(read_size)
	#if not data:
	#	return None
	#evdev_objects = iter_unpack(data)
	#events = [self._make_event(*event) for event in evdev_objects]
#	return None

async def read_without_blocking():
	return get_gamepad()

task = asyncio.create_task(read_without_blocking())

while True:
	#print(devices.gamepads[0]._character_device_path)
	#print(devices.gamepads[0]._character_device.tell())
	if (task.done()):
		exit()
	#events = get_gamepad()
	#for e in events:
	#	print(e.code, e.state, end="")
	#for e in my_do_iter(devices.gamepads[0]):
	#	print(e.code, e.state, end="")
	print("time :",time.time(), end="\r")
