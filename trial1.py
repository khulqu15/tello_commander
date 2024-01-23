from djitellopy import Tello
import argparse

def main(drone_id):
    tello = Tello()
    tello.connect()
    tello.takeoff()
    tello.move_forward(150)

    if drone_id == 1:
        tello.move_right(150)
        tello.move_up(20)
        tello.move_right(100)
    elif drone_id == 2:
        tello.move_right(150)
        tello.move_up(20)
        tello.move_right(100)
    else:
        print("Invalid drone id")
        exit()

    tello.move_forward(100)
    tello.land()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone Control")
    parser.add_argument("--drone", help="Drone ID", default=None)
    args = parser.parse_args()
    drone = args.drone if args.drone is not None else int(input("Enter drone id [1|2] : "))
    main(drone)
