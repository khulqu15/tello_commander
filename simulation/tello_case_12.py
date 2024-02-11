from djitellopy import Tello
import pyrebase
import argparse
import time
import math
    
def run(database, drone_id):
    start_time = time.time()
    max_operation_time = 5000
    safe_distance = 10
    safe_altitude = 150
    tello = Tello()
    tello.connect()
    tello.takeoff()
    tello.move_up(50)
    target_x, target_y = 380, 280 if drone_id == 1 else 20, 20
    move_to(tello, target_x, target_y, drone_id)
    while True:
        if time.time() - start_time > max_operation_time:
            print("Operational Time Ended")
            tello.land()
            break
        
        own_data = save_data(tello, database, drone_id)
        other_data = get_other_drone_data(database, drone_id)
        distance = math.sqrt((other_data[0] - own_data[0]) ** 2 + (other_data[1] - own_data[1]) ** 2)
        current_altitude = tello.get_height()
        if distance < safe_distance:
            if current_altitude < safe_distance:
                tello.move_up(10)
            else:
                tello.move_down(10)
        else:
            if current_altitude > safe_distance:
                tello.move_down(10)
                
        if distance < safe_distance: tello.send_rc_control(0, 0, -10, 0)
        else: tello.send_rc_control(0, 0, 10, 0)
        if math.sqrt((target_x - own_data[0]) ** 2 + (target_y - own_data[1]) ** 2) < safe_distance:
            print("Tujuan tercapai, mendarat.")
            tello.land()
            break
    
def move_to(tello: Tello, x, y, drone_id):
    move_step = 5
    current_x, current_y = 20, 20 if drone_id == 1 else 380, 280
    while current_x != x or current_y != y:
        if current_x < x:
            tello.move_forward(min(move_step, x - current_x))
            current_x += move_step
        if current_x > x:
            tello.move_back(min(move_step, current_x - x))
            current_x -= move_step
        if current_y < y:
            tello.move_right(min(move_step, y - current_y))
            current_y += move_step
        if current_y > y:
            tello.move_left(min(move_step, current_y - y))
            current_y -= move_step
        
def save_data(tello: Tello, database, drone_id):
    x = tello.get_acceleration_x()
    y = tello.get_acceleration_y()
    z = tello.get_acceleration_z()
    roll = tello.get_roll()
    pitch = tello.get_pitch()
    yaw = tello.get_yaw()
    data = str(x) + ',' + str(y) + ',' + str(z) + ',' + str(roll) + ',' + str(pitch) + ',' + str(yaw)
    data_drone = database.child(f"menu/tello/data/drone{drone_id}").shallow().get()
    length_data = len(data_drone.val())
    database.child(f"menu/tello/data/drone{drone_id}/{length_data}").set(data)
    return [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]
    
def get_other_drone_data(database, drone_id):
    other_drone = 1 if drone_id == 1 else 2
    data_drone = database.child(f"menu/tello/data/drone{other_drone}").shallow().get()
    length_data = len(data_drone.val())
    newest_data = database.child(f"menu/tello/data/drone{other_drone}/{length_data-1}").get()
    split_data = str(newest_data.val()).split(",")
    x = float(split_data[0])
    y = float(split_data[1])
    z = float(split_data[2])
    roll = float(split_data[3])
    pitch = float(split_data[4])
    yaw = float(split_data[5])
    return [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]
    

def main(email, password, drone):
    config = {
        "apiKey": "AIzaSyBYoGN_ayC7GFWIeTSdKkh4vl022YYJEqw",
        "authDomain": "hayago-42fbf.firebaseapp.com",
        "databaseURL": "https://hayago-42fbf-default-rtdb.asia-southeast1.firebasedatabase.app",
        "storageBucket": "hayago-42fbf.appspot.com",
        "projectId": "hayago-42fbf",
        "messagingSenderId": "806345285802",
        "appId": "1:806345285802:web:872dc1e96765093dff30d3",
        "measurementId": "G-S2ZSFSY0JQ"
    }

    firebase = pyrebase.initialize_app(config=config)
    db = firebase.database()
    auth = firebase.auth()

    try:
        auth.sign_in_with_email_and_password(email, password)
        print("User signed in successfully.")
    except Exception as error:
        print(f"Error signing user: {error}")
        try:
            auth.create_user_with_email_and_password(email, password)
            print("User created successfully.")
        except Exception as error:
            print(f"Error creating user: {error}")
    
    run(db, drone)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone Control")
    parser.add_argument("--email", help="Email address", default=None)
    parser.add_argument("--password", help="Password", default=None)
    parser.add_argument("--drone", help="Drone ID", default=None)
    args = parser.parse_args()
    email = args.email if args.email else input("Enter email : ")
    password = args.password if args.password else input("Enter password : ")
    drone = args.drone if args.drone is not None else int(input("Enter drone id [1|2] : "))
    main(email, password, drone)