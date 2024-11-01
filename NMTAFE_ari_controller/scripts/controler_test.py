from controler import Controller
import time

gamePad = Controller(background_polling=True)

while True:
	print(gamePad.left_joy.direction, end=" ")
	print(time.time(), end="\r")