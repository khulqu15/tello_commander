#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from tello_msgs.srv import TelloAction
import time
import math
import matplotlib.pyplot as plt
import random
from mpl_toolkits.mplot3d import axes3d
import csv
# import keyboard

class TelloCommander(Node):
    def __init__(self):
        super().__init__('tello_commander')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.service_client = self.create_client(TelloAction, '/drone1/tello_action')
        # self.create_subscription(Twist, '/drone1/cmd_vel', self.twist_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/drone1/odom', self.odom_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/drone1/pose', self.pose_callback, 10)
        self.speed = 0.15
        self.turn = 0.15
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.uwb_x = 0.0
        self.uwb_y = 0.0
        self.uwb_z = 0.0
        self.status = 0.0
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.th_history = []
        self.uwb_x_history = []
        self.uwb_y_history = []
        self.uwb_z_history = []
        
    def log_data(self):
        self.x_history.append(self.x)
        self.y_history.append(self.y)
        self.z_history.append(self.z)
        self.th_history.append(self.th)
        self.uwb_x_history.append(self.uwb_x)
        self.uwb_y_history.append(self.uwb_y)
        self.uwb_z_history.append(self.uwb_z)

    def save_to_csv(self, file_name='drone_trajectory_data.csv'):
        with open(file_name, mode="w", newline="") as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["x", "y", "z", "th", "uwb_x", "uwb_y", "uwb_z"])
            for i in range(len(self.x_history)):
                csv_writer.writerow([
                    self.x_history[i],
                    self.y_history[i],
                    self.z_history[i],
                    self.th_history[i],
                    self.uwb_x_history[i],
                    self.uwb_y_history[i],
                    self.uwb_z_history[i]
                ])
    
    def calculate_uwb(self):
        A = (5, -4)
        B = (1, -2)
        C = (5, 4)
        self.uwb_x = self.x - (random.randint(0, 3) / 10)
        self.uwb_y = self.y - (random.randint(0, 3) / 10)
        self.uwb_z = self.z - (random.randint(0, 3) / 10)
        self.uwb_x_history.append(self.uwb_x)
        self.uwb_y_history.append(self.uwb_y)
        self.uwb_z_history.append(self.uwb_z)

    def send_cmd_vel(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x * self.speed
        msg.linear.y = linear_y * self.speed
        msg.linear.z = linear_z * self.speed
        msg.angular.x = angular_x * self.turn
        msg.angular.y = angular_y * self.turn
        msg.angular.z = angular_z * self.turn
        self.cmd_vel_publisher.publish(msg)

    def call_tello_action(self, command):
        request = TelloAction.Request()
        request.cmd = command
        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result()}')
        else:
            self.get_logger().warning('Service call failed: %r' % (future.exception(),))

    def fly_forward_and_turn(self):
        self.send_cmd_vel(linear_y=-1)
        self.print_info()
        self.get_logger().info('Flying forward and left...')
        self.log_data()
        time.sleep(3)
        rclpy.spin_once(self, timeout_sec=5)
        self.print_info()
        self.send_cmd_vel(linear_x=1)
        self.log_data()
        time.sleep(2)
        rclpy.spin_once(self, timeout_sec=5)
        self.get_logger().info('Get data 200 times...')
        for i in range(50):
            self.get_logger().info(f'Get data flight: ({i})')
            self.print_info()
            self.send_cmd_vel(linear_y=1, linear_z=0.1 + (random.randint(-4, 4) / 100))
            self.log_data()
            time.sleep(1)
            rclpy.spin_once(self, timeout_sec=5)
            self.print_info()
            self.send_cmd_vel(linear_x=1)
            self.log_data()
            time.sleep(1 + (i / 200))
            rclpy.spin_once(self, timeout_sec=5)
            self.print_info()
            self.send_cmd_vel(linear_y=-1)
            self.log_data()
            time.sleep(1)
            rclpy.spin_once(self, timeout_sec=5)
            self.print_info()
            self.send_cmd_vel(linear_x=-1)
            self.log_data()
            time.sleep(1)
            rclpy.spin_once(self, timeout_sec=5)
            # if keyboard.is_pressed('q'):
            #     print("q key pressed, breaking the loop...")
            #     break
            
        self.send_cmd_vel()
        self.get_logger().info('Stopping...')
        rclpy.spin_once(self, timeout_sec=5)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.th = msg.pose.pose.orientation.z
        self.x_history.append(self.x)
        self.y_history.append(self.y)
        self.z_history.append(self.z)
        self.th_history.append(self.th)
        self.calculate_uwb()

    def pose_callback(self, msg):
        self.status = msg.header.stamp.sec

    def print_info(self):
        self.get_logger().info(f'Location: ({self.x}, {self.y}, {self.z}), UWB: ({self.uwb_x}, {self.uwb_y}, {self.z}), Orientation: {self.th}, Status: {self.status}')

    def run(self):
        self.log_data()
        self.call_tello_action('takeoff')
        rclpy.spin_once(self, timeout_sec=5)
        self.log_data()
        time.sleep(5)
        self.print_info()
        self.fly_forward_and_turn()
        self.log_data()
        time.sleep(5)
        self.print_info()
        self.call_tello_action('land')
        self.log_data()
        rclpy.spin_once(self, timeout_sec=5)
        self.print_info()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(self.x_history, self.y_history, self.z_history, label="Copter Trajectory (Actual)")
        ax.plot(self.uwb_x_history, self.uwb_y_history, self.uwb_z_history, label="Copter Trajectory (UWB)", color="green")
        ax.quiver(self.x_history, self.y_history, self.z_history, 0, 0, self.th_history, length=0.1, normalize=True, color="red", label="Drone Yaw Orientation")
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.legend()
        self.save_to_csv()
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    commander = TelloCommander()
    commander.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
