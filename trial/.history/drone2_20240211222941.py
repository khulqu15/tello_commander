from djitellopy import Tello
import time
from firebase import database
import math

print("______________________________________")
print("Tello cases")
print("[1] Case 1")
print("[2] Case 2")
print("[3] Case 3")
print("[4] Case 4")
print("Choose a case:")
case = input()

x_pos = 0.0
y_pos = 0.0
z_pos = 0.0
roll_pos = 0.0
pitch_pos = 0.0
yaw_pos = 0.0

def upload():
    coordinate = str(x_pos)+","+str(y_pos)+","+str(z_pos)+","+str(roll_pos)+","+str(pitch_pos)+","+str(yaw_pos)
    data = database.child("menu").child("tello").child("data").child("drone1").get()
    index = len(data.val())
    database.child("menu").child("tello").child("data").child("drone1").child(index).set(coordinate)

tello = Tello()
tello.connect()
tello.takeoff()
z_pos = tello.get_acceleration_z()
upload()

if int(case) == 1 or int(case) == 2 or int(case) == 4:
    tello.move_forward(140)
    x_pos = 140
    upload()
    time.sleep(2)

    tello.move_right(180)
    y_pos = 180
    upload()
    
    time.sleep(2)
    tello.move_up(5)
    z_pos += 5
    upload()
    
    time.sleep(2)
    tello.move_right(180)
    y_pos += 180
    upload()

    time.sleep(2)
    tello.move_forward(140)
    x_pos += 140
    upload()
    
    tello.rotate_counter_clockwise(180)

if int(case) == 3:
    tello.move_forward(100)
    x_pos = 100
    upload()
    
    tello.move_right(10)
    y_pos = 10
    upload()
    
    for i in range(360):
        tello.move_forward(1)
        tello.rotate_clockwise(1)
        x_pos += math.cos(math.radians(i))
        x_pos += math.sin(math.radians(i))
        upload()
        
tello.land()
z_pos = 0
x_pos = 0
y_pos = 0
upload()