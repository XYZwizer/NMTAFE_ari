from controler import Controller
import time

gamePad = Controller(background_polling=True)

while True:
	try:
		print(gamePad.left_joy.direction, end=" ")
		print(gamePad.north, end=" ")
		print(time.time(), end="\r")
	except KeyboardInterrupt:
		#gamePad.background_polling.kill()
		exit(0)
