from djitellopy import Tello
import time
from firebase import database, user, auth

print("______________________________________")
print("Choose a case:")
print("[1] Case 1")
print("[2] Case 2")
print("[3] Case 3")
print("[4] Case 4")
case = input()

tello = Tello()
tello.connect()
tello.takeoff()

if int(case) == 1 or int(case) == 2:
    tello.move_forward(140)
    time.sleep(2)

    tello.move_right(180)
    time.sleep(2)
    tello.move_up(5)
    time.sleep(2)
    tello.move_right(180)

    time.sleep(2)
    tello.move_forward(140)
    tello.rotate_counter_clockwise(180)

tello.land()    