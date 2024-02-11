from djitellopy import Tello
import time
from firebase import database

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

def upload(x, y, z, roll, pitch, yaw):
    x_pos = x
    y_pos = y
    z_pos = z
    coordinate = str(x_pos)+","+str(y_pos)+","+str(z_pos)+","+str(roll_pos)+","+str(pitch_pos)+","+str(yaw_pos)
    data = database.child("menu").child("tello").child("data").child("drone1").get()
    index = len(data.val())
    database.child("menu").child("tello").child("data").child("drone1").child(index).set(coordinate)

# tello = Tello()
# tello.connect()
# tello.takeoff()

# if int(case) == 1 or int(case) == 2 or int(case) == 4:
#     tello.move_forward(140)
#     time.sleep(2)

#     tello.move_right(180)
#     time.sleep(2)
#     tello.move_up(5)
#     time.sleep(2)
#     tello.move_right(180)

#     time.sleep(2)
#     tello.move_forward(140)
#     tello.rotate_counter_clockwise(180)

# if int(case) == 3:
#     tello.move_forward(100)
#     tello.move_right(10)
    
#     for i in range(360):
#         tello.move_forward(1)
#         tello.rotate_clockwise(1)
        
# tello.land()    

upload()