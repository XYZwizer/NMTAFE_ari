from controler import Controller
import time

gamePad = Controller(background_polling=True)
old_dir = (0,0)
new_dir = (0,0)
while True:
	try:
		new_dir = gamePad.left_joy.direction
		if new_dir != old_dir:
			print(new_dir, end="\n")
			old_dir = new_dir
		#print(gamePad.north, end=" ")
		if gamePad.north:
			gamePad.left_joy.setCurrentAsDeadZone()
		if gamePad.west:
			gamePad.left_joy.findMax()
			print("new_max")
		#print(time.time(), end="\n")
	except KeyboardInterrupt:
		#gamePad.background_polling.kill()
		exit(0)
