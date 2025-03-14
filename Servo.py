import pygame
import numpy as np
import time
from pymata4 import pymata4

# Initialize Arduino board
board = pymata4.Pymata4()

SERVO_PIN = 9
board.set_pin_mode_servo(SERVO_PIN)
board.servo_write(9, 0)
time.sleep(1)
board.servo_write(9, 180)


