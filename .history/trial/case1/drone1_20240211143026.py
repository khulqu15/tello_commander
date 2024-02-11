from djitellopy import Tello
import pyrebase

tello = Tello()
tello.connect()
tello.takeoff()

tello.move_forward(100)
tello.move_right(100)
tello.move_forward(100)

tello.rotate_counter_clockwise(180)
