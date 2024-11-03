from controler import Controller
import time

gamePad = Controller(background_polling=True)

while True:
	try:
		print(gamePad.left_joy.direction, end="\n")
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
