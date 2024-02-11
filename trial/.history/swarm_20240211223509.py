from djitellopy import TelloSwarm
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
    data = database.child("menu").child("tello").child("data").child("drone2").get()
    data = database.child("menu").child("tello").child("data").child("drone1").get()
    index = len(data.val())
    database.child("menu").child("tello").child("data").child("drone2").child(index).set(coordinate)
    database.child("menu").child("tello").child("data").child("drone1").child(index).set(coordinate)

swarm = TelloSwarm.fromIps([
    "192.168.178.42",
    "192.168.178.43",
])
swarm.connect()
swarm.takeoff()
z_pos = 10
upload()

swarm.sequential(lambda i, tello: tello.move_forward(i * 20 + 20))
swarm.parallel(lambda i, tello: tello.move_left(i * 100 + 20))

swarm.land()
swarm.end()