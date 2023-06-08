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
import pandas as pd
import csv
# import keyboard

class TelloCommander(Node):
    def __init__(self):
        super().__init__('tello_commander')
        self.cmd_vel_publisher1 = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.cmd_vel_publisher2 = self.create_publisher(Twist, '/drone2/cmd_vel', 10)
        self.cmd_vel_publisher3 = self.create_publisher(Twist, '/drone3/cmd_vel', 10)
        self.cmd_vel_publisher4 = self.create_publisher(Twist, '/drone4/cmd_vel', 10)
        self.cmd_vel_publisher5 = self.create_publisher(Twist, '/drone5/cmd_vel', 10)
        self.service_client1 = self.create_client(TelloAction, '/drone1/tello_action')
        self.service_client2 = self.create_client(TelloAction, '/drone2/tello_action')
        self.service_client3 = self.create_client(TelloAction, '/drone3/tello_action')
        self.service_client4 = self.create_client(TelloAction, '/drone4/tello_action')
        self.service_client5 = self.create_client(TelloAction, '/drone5/tello_action')
        # self.create_subscription(Twist, '/drone1/cmd_vel', self.twist_callback, 10)
        self.odom_subscriber1 = self.create_subscription(Odometry, '/drone1/odom', self.odom_callback1, 10)
        self.odom_subscriber2 = self.create_subscription(Odometry, '/drone2/odom', self.odom_callback2, 10)
        self.odom_subscriber3 = self.create_subscription(Odometry, '/drone3/odom', self.odom_callback3, 10)
        self.odom_subscriber4 = self.create_subscription(Odometry, '/drone4/odom', self.odom_callback4, 10)
        self.odom_subscriber5 = self.create_subscription(Odometry, '/drone5/odom', self.odom_callback5, 10)
        self.pose_subscriber1 = self.create_subscription(PoseStamped, '/drone1/pose', self.pose_callback1, 10)
        self.pose_subscriber2 = self.create_subscription(PoseStamped, '/drone2/pose', self.pose_callback2, 10)
        self.pose_subscriber3 = self.create_subscription(PoseStamped, '/drone3/pose', self.pose_callback3, 10)
        self.pose_subscriber4 = self.create_subscription(PoseStamped, '/drone4/pose', self.pose_callback4, 10)
        self.pose_subscriber5 = self.create_subscription(PoseStamped, '/drone5/pose', self.pose_callback5, 10)
        self.altitude = 2
        self.speed = 0.5
        self.turn = 1.5
        self.drone1_location = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'th': 0.0,
            'uwb_x': 0.0,
            'uwb_y': 0.0,
            'uwb_z': 0.0,
            'status': 0.0,
            'x_history': [],
            'y_history': [],
            'th_history': [],
            'z_history': [],
            'uwb_x_history': [],
            'uwb_y_history': [],
            'uwb_z_history': [],
            'distance_history': [],
        }
        self.drone2_location = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'th': 0.0,
            'uwb_x': 0.0,
            'uwb_y': 0.0,
            'uwb_z': 0.0,
            'status': 0.0,
            'x_history': [],
            'y_history': [],
            'th_history': [],
            'z_history': [],
            'uwb_x_history': [],
            'uwb_y_history': [],
            'uwb_z_history': [],
            'distance_history': [],
        }
        self.drone3_location = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'th': 0.0,
            'uwb_x': 0.0,
            'uwb_y': 0.0,
            'uwb_z': 0.0,
            'status': 0.0,
            'x_history': [],
            'y_history': [],
            'th_history': [],
            'z_history': [],
            'uwb_x_history': [],
            'uwb_y_history': [],
            'uwb_z_history': [],
            'distance_history': [],
        }
        self.drone4_location = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'th': 0.0,
            'uwb_x': 0.0,
            'uwb_y': 0.0,
            'uwb_z': 0.0,
            'status': 0.0,
            'x_history': [],
            'y_history': [],
            'th_history': [],
            'z_history': [],
            'uwb_x_history': [],
            'uwb_y_history': [],
            'uwb_z_history': [],
            'distance_history': [],
        }
        self.drone5_location = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'th': 0.0,
            'uwb_x': 0.0,
            'uwb_y': 0.0,
            'uwb_z': 0.0,
            'status': 0.0,
            'x_history': [],
            'y_history': [],
            'th_history': [],
            'z_history': [],
            'uwb_x_history': [],
            'uwb_y_history': [],
            'uwb_z_history': [],
            'distance_history': [],
        }

    def log_data(self):
        rclpy.spin_once(self, timeout_sec=5)
        
        self.drone1_location['x_history'].append(self.drone1_location['x'])
        self.drone1_location['y_history'].append(self.drone1_location['y'])
        self.drone1_location['z_history'].append(self.drone1_location['z'])
        self.drone1_location['th_history'].append(self.drone1_location['th'])
        self.drone1_location['uwb_x_history'].append(self.drone1_location['uwb_x'])
        self.drone1_location['uwb_y_history'].append(self.drone1_location['uwb_y'])
        self.drone1_location['uwb_z_history'].append(self.drone1_location['uwb_z'])
        distance1 = math.sqrt((self.drone1_location['x'] - self.drone1_location['uwb_x'])**2 + (self.drone1_location['y'] - self.drone1_location['uwb_y'])**2 + (self.drone1_location['z'] - self.drone1_location['uwb_z'])**2)
        self.drone1_location['distance_history'].append(distance1)
        
        self.drone2_location['x_history'].append(self.drone2_location['x'])
        self.drone2_location['y_history'].append(self.drone2_location['y'])
        self.drone2_location['z_history'].append(self.drone2_location['z'])
        self.drone2_location['th_history'].append(self.drone2_location['th'])
        self.drone2_location['uwb_x_history'].append(self.drone2_location['uwb_x'])
        self.drone2_location['uwb_y_history'].append(self.drone2_location['uwb_y'])
        self.drone2_location['uwb_z_history'].append(self.drone2_location['uwb_z'])
        distance2 = math.sqrt((self.drone2_location['x'] - self.drone2_location['uwb_x'])**2 + (self.drone2_location['y'] - self.drone2_location['uwb_y'])**2 + (self.drone2_location['z'] - self.drone2_location['uwb_z'])**2)
        self.drone2_location['distance_history'].append(distance2)

        self.drone3_location['x_history'].append(self.drone3_location['x'])
        self.drone3_location['y_history'].append(self.drone3_location['y'])
        self.drone3_location['z_history'].append(self.drone3_location['z'])
        self.drone3_location['th_history'].append(self.drone3_location['th'])
        self.drone3_location['uwb_x_history'].append(self.drone3_location['uwb_x'])
        self.drone3_location['uwb_y_history'].append(self.drone3_location['uwb_y'])
        self.drone3_location['uwb_z_history'].append(self.drone3_location['uwb_z'])
        distance3 = math.sqrt((self.drone3_location['x'] - self.drone3_location['uwb_x'])**2 + (self.drone3_location['y'] - self.drone3_location['uwb_y'])**2 + (self.drone3_location['z'] - self.drone3_location['uwb_z'])**2)
        self.drone3_location['distance_history'].append(distance3)
        
        self.drone4_location['x_history'].append(self.drone4_location['x'])
        self.drone4_location['y_history'].append(self.drone4_location['y'])
        self.drone4_location['z_history'].append(self.drone4_location['z'])
        self.drone4_location['th_history'].append(self.drone4_location['th'])
        self.drone4_location['uwb_x_history'].append(self.drone4_location['uwb_x'])
        self.drone4_location['uwb_y_history'].append(self.drone4_location['uwb_y'])
        self.drone4_location['uwb_z_history'].append(self.drone4_location['uwb_z'])
        distance4 = math.sqrt((self.drone4_location['x'] - self.drone4_location['uwb_x'])**2 + (self.drone4_location['y'] - self.drone4_location['uwb_y'])**2 + (self.drone4_location['z'] - self.drone4_location['uwb_z'])**2)
        self.drone4_location['distance_history'].append(distance4)
        
        self.drone5_location['x_history'].append(self.drone5_location['x'])
        self.drone5_location['y_history'].append(self.drone5_location['y'])
        self.drone5_location['z_history'].append(self.drone5_location['z'])
        self.drone5_location['th_history'].append(self.drone5_location['th'])
        self.drone5_location['uwb_x_history'].append(self.drone5_location['uwb_x'])
        self.drone5_location['uwb_y_history'].append(self.drone5_location['uwb_y'])
        self.drone5_location['uwb_z_history'].append(self.drone5_location['uwb_z'])
        distance5 = math.sqrt((self.drone5_location['x'] - self.drone5_location['uwb_x'])**2 + (self.drone5_location['y'] - self.drone5_location['uwb_y'])**2 + (self.drone5_location['z'] - self.drone5_location['uwb_z'])**2)
        self.drone5_location['distance_history'].append(distance5)


    def save_to_csv(self, file_name='drone_trajectory_data.csv'):
        for drone, data in enumerate([self.drone1_location, self.drone2_location, self.drone3_location, self.drone4_location, self.drone5_location], start=1):
            df = pd.DataFrame({
                "x": data['x_history'],
                "y": data['y_history'],
                "z": data['z_history'],
                "th": data['th_history'],
                "uwb_x": data['uwb_x_history'],
                "uwb_y": data['uwb_y_history'],
                "uwb_z": data['uwb_z_history'],
                "distance": data['distance_history']
            })
            csv_file = f'drone{drone}_{file_name}'
            df.to_csv(csv_file, index=False)

    def calculate_uwb(self, drone_number):
        A = (5, -4)
        B = (1, -2)
        C = (5, 4)
        
        if drone_number == 1:
            self.drone1_location['uwb_x'] = self.drone1_location['x'] - (random.randint(-2, 3) / 100)
            self.drone1_location['uwb_y'] = self.drone1_location['y'] - (random.randint(-2, 3) / 100)
            self.drone1_location['uwb_z'] = self.drone1_location['z'] - (random.randint(-2, 3) / 100)
            self.drone1_location['uwb_x_history'].append(self.drone1_location['uwb_x'])
            self.drone1_location['uwb_y_history'].append(self.drone1_location['uwb_y'])
            self.drone1_location['uwb_z_history'].append(self.drone1_location['uwb_z'])
            distance = math.sqrt((self.drone1_location['x'] - self.drone1_location['uwb_x'])**2 + (self.drone1_location['y'] - self.drone1_location['uwb_y'])**2 + (self.drone1_location['z'] - self.drone1_location['uwb_z'])**2)
            self.drone1_location['distance_history'].append(distance)
        
        if drone_number == 2:
            self.drone2_location['uwb_x'] = self.drone2_location['x'] - (random.randint(-2, 3) / 100)
            self.drone2_location['uwb_y'] = self.drone2_location['y'] - (random.randint(-2, 3) / 100)
            self.drone2_location['uwb_z'] = self.drone2_location['z'] - (random.randint(-2, 3) / 100)
            self.drone2_location['uwb_x_history'].append(self.drone2_location['uwb_x'])
            self.drone2_location['uwb_y_history'].append(self.drone2_location['uwb_y'])
            self.drone2_location['uwb_z_history'].append(self.drone2_location['uwb_z'])
            distance = math.sqrt((self.drone2_location['x'] - self.drone2_location['uwb_x'])**2 + (self.drone2_location['y'] - self.drone2_location['uwb_y'])**2 + (self.drone2_location['z'] - self.drone2_location['uwb_z'])**2)
            self.drone2_location['distance_history'].append(distance)
        
        if drone_number == 3:
            self.drone3_location['uwb_x'] = self.drone3_location['x'] - (random.randint(-2, 3) / 100)
            self.drone3_location['uwb_y'] = self.drone3_location['y'] - (random.randint(-2, 3) / 100)
            self.drone3_location['uwb_z'] = self.drone3_location['z'] - (random.randint(-2, 3) / 100)
            self.drone3_location['uwb_x_history'].append(self.drone3_location['uwb_x'])
            self.drone3_location['uwb_y_history'].append(self.drone3_location['uwb_y'])
            self.drone3_location['uwb_z_history'].append(self.drone3_location['uwb_z'])
            distance = math.sqrt((self.drone3_location['x'] - self.drone3_location['uwb_x'])**2 + (self.drone3_location['y'] - self.drone3_location['uwb_y'])**2 + (self.drone3_location['z'] - self.drone3_location['uwb_z'])**2)
            self.drone3_location['distance_history'].append(distance)
        
        if drone_number == 4:
            self.drone4_location['uwb_x'] = self.drone4_location['x'] - (random.randint(-2, 3) / 100)
            self.drone4_location['uwb_y'] = self.drone4_location['y'] - (random.randint(-2, 3) / 100)
            self.drone4_location['uwb_z'] = self.drone4_location['z'] - (random.randint(-2, 3) / 100)
            self.drone4_location['uwb_x_history'].append(self.drone4_location['uwb_x'])
            self.drone4_location['uwb_y_history'].append(self.drone4_location['uwb_y'])
            self.drone4_location['uwb_z_history'].append(self.drone4_location['uwb_z'])
            distance = math.sqrt((self.drone4_location['x'] - self.drone4_location['uwb_x'])**2 + (self.drone4_location['y'] - self.drone4_location['uwb_y'])**2 + (self.drone4_location['z'] - self.drone4_location['uwb_z'])**2)
            self.drone4_location['distance_history'].append(distance)
        
        if drone_number == 5:
            self.drone5_location['uwb_x'] = self.drone5_location['x'] - (random.randint(-2, 3) / 100)
            self.drone5_location['uwb_y'] = self.drone5_location['y'] - (random.randint(-2, 3) / 100)
            self.drone5_location['uwb_z'] = self.drone5_location['z'] - (random.randint(-2, 3) / 100)
            self.drone5_location['uwb_x_history'].append(self.drone5_location['uwb_x'])
            self.drone5_location['uwb_y_history'].append(self.drone5_location['uwb_y'])
            self.drone5_location['uwb_z_history'].append(self.drone5_location['uwb_z'])
            distance = math.sqrt((self.drone5_location['x'] - self.drone5_location['uwb_x'])**2 + (self.drone5_location['y'] - self.drone5_location['uwb_y'])**2 + (self.drone5_location['z'] - self.drone5_location['uwb_z'])**2)
            self.drone5_location['distance_history'].append(distance)
            

    def send_cmd_vel(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x * self.speed
        msg.linear.y = linear_y * self.speed
        msg.linear.z = linear_z * self.speed
        msg.angular.x = angular_x * self.turn
        msg.angular.y = angular_y * self.turn
        msg.angular.z = angular_z * self.turn
        self.cmd_vel_publisher1.publish(msg)
        self.cmd_vel_publisher2.publish(msg)
        self.cmd_vel_publisher3.publish(msg)
        self.cmd_vel_publisher4.publish(msg)
        self.cmd_vel_publisher5.publish(msg)

    def call_tello_action(self, command):
        request = TelloAction.Request()
        request.cmd = command
        future = self.service_client1.call_async(request)
        future = self.service_client2.call_async(request)
        future = self.service_client3.call_async(request)
        future = self.service_client4.call_async(request)
        future = self.service_client5.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result()}')
        else:
            self.get_logger().warning('Service call failed: %r' % (future.exception(),))

    def fly_forward_and_turn(self):
        self.send_cmd_vel(linear_y=-1)
        rclpy.spin_once(self, timeout_sec=5)
        self.print_info()
        self.get_logger().info('Flying forward and left...')
        self.log_data()
        time.sleep(self.altitude)
        rclpy.spin_once(self, timeout_sec=5)
        self.print_info()
        self.send_cmd_vel(linear_x=1)
        rclpy.spin_once(self, timeout_sec=5)
        self.log_data()
        time.sleep(self.altitude)
        rclpy.spin_once(self, timeout_sec=5)
        self.get_logger().info('Get data 200 times...')
        for i in range(10):
            self.get_logger().info(f'Get data flight: ({i})')
            self.print_info()
            self.send_cmd_vel(linear_y=-1)
            rclpy.spin_once(self, timeout_sec=5)
            self.log_data()
            time.sleep(self.altitude)
            rclpy.spin_once(self, timeout_sec=5)
            self.print_info()
            self.send_cmd_vel(linear_x=1)
            rclpy.spin_once(self, timeout_sec=5)
            self.log_data()
            time.sleep(self.altitude)
            rclpy.spin_once(self, timeout_sec=5)
            self.print_info()
            self.send_cmd_vel(linear_y=1)
            rclpy.spin_once(self, timeout_sec=5)
            self.log_data()
            time.sleep(self.altitude)
            rclpy.spin_once(self, timeout_sec=5)
            self.print_info()
            self.send_cmd_vel(linear_x=-1)
            self.log_data()
            time.sleep(self.altitude)
            rclpy.spin_once(self, timeout_sec=5)
            # if keyboard.is_pressed('q'):
            #     print("q key pressed, breaking the loop...")
            #     break
            
        self.send_cmd_vel()
        self.get_logger().info('Stopping...')
        rclpy.spin_once(self, timeout_sec=5)

    def odom_callback1(self, msg):
        self.drone1_location['x'] = msg.pose.pose.position.x
        self.drone1_location['y'] = msg.pose.pose.position.y
        self.drone1_location['z'] = msg.pose.pose.position.z - 0.5
        self.drone1_location['th'] = msg.pose.pose.orientation.z
        self.drone1_location['x_history'].append(self.drone1_location['x'])
        self.drone1_location['y_history'].append(self.drone1_location['y'])
        self.drone1_location['z_history'].append(self.drone1_location['z'])
        self.drone1_location['th_history'].append(self.drone1_location['th'])
        self.calculate_uwb(1)
        
    def odom_callback2(self, msg):
        self.drone2_location['x'] = msg.pose.pose.position.x
        self.drone2_location['y'] = msg.pose.pose.position.y
        self.drone2_location['z'] = msg.pose.pose.position.z - 0.5
        self.drone2_location['th'] = msg.pose.pose.orientation.z
        self.drone2_location['x_history'].append(self.drone2_location['x'])
        self.drone2_location['y_history'].append(self.drone2_location['y'])
        self.drone2_location['z_history'].append(self.drone2_location['z'])
        self.drone2_location['th_history'].append(self.drone2_location['th'])
        self.calculate_uwb(2)
        
    def odom_callback3(self, msg):
        self.drone3_location['x'] = msg.pose.pose.position.x
        self.drone3_location['y'] = msg.pose.pose.position.y
        self.drone3_location['z'] = msg.pose.pose.position.z - 0.5
        self.drone3_location['th'] = msg.pose.pose.orientation.z
        self.drone3_location['x_history'].append(self.drone3_location['x'])
        self.drone3_location['y_history'].append(self.drone3_location['y'])
        self.drone3_location['z_history'].append(self.drone3_location['z'])
        self.drone3_location['th_history'].append(self.drone3_location['th'])
        self.calculate_uwb(3)
        
    def odom_callback4(self, msg):
        self.drone4_location['x'] = msg.pose.pose.position.x
        self.drone4_location['y'] = msg.pose.pose.position.y
        self.drone4_location['z'] = msg.pose.pose.position.z - 0.5
        self.drone4_location['th'] = msg.pose.pose.orientation.z
        self.drone4_location['x_history'].append(self.drone4_location['x'])
        self.drone4_location['y_history'].append(self.drone4_location['y'])
        self.drone4_location['z_history'].append(self.drone4_location['z'])
        self.drone4_location['th_history'].append(self.drone4_location['th'])
        self.calculate_uwb(4)
        
    def odom_callback5(self, msg):
        self.drone5_location['x'] = msg.pose.pose.position.x
        self.drone5_location['y'] = msg.pose.pose.position.y
        self.drone5_location['z'] = msg.pose.pose.position.z - 0.5
        self.drone5_location['th'] = msg.pose.pose.orientation.z
        self.drone5_location['x_history'].append(self.drone5_location['x'])
        self.drone5_location['y_history'].append(self.drone5_location['y'])
        self.drone5_location['z_history'].append(self.drone5_location['z'])
        self.drone5_location['th_history'].append(self.drone5_location['th'])
        self.calculate_uwb(5)

    def pose_callback1(self, msg):
        self.drone1_location['status'] = msg.header.stamp.sec
        
    def pose_callback2(self, msg):
        self.drone2_location['status'] = msg.header.stamp.se
    
    def pose_callback3(self, msg):
        self.drone3_location['status'] = msg.header.stamp.sec

    def pose_callback4(self, msg):
        self.drone4_location['status'] = msg.header.stamp.sec

    def pose_callback5(self, msg):
        self.drone5_location['status'] = msg.header.stamp.sec

    def print_info(self):
        self.get_logger().info(f"Location Drone 1: ({self.drone1_location['x']}, {self.drone1_location['y']}, {self.drone1_location['z']}), UWB: ({self.drone1_location['uwb_x']}, {self.drone1_location['uwb_y']}, {self.drone1_location['uwb_z']}), Orientation: {self.drone1_location['th']}, Status: {self.drone1_location['status']}")
        self.get_logger().info(f"Location Drone 2: ({self.drone2_location['x']}, {self.drone2_location['y']}, {self.drone2_location['z']}), UWB: ({self.drone2_location['uwb_x']}, {self.drone2_location['uwb_y']}, {self.drone2_location['uwb_z']}), Orientation: {self.drone2_location['th']}, Status: {self.drone2_location['status']}")
        self.get_logger().info(f"Location Drone 3: ({self.drone3_location['x']}, {self.drone3_location['y']}, {self.drone3_location['z']}), UWB: ({self.drone3_location['uwb_x']}, {self.drone3_location['uwb_y']}, {self.drone3_location['uwb_z']}), Orientation: {self.drone3_location['th']}, Status: {self.drone3_location['status']}")
        self.get_logger().info(f"Location Drone 4: ({self.drone4_location['x']}, {self.drone4_location['y']}, {self.drone4_location['z']}), UWB: ({self.drone4_location['uwb_x']}, {self.drone4_location['uwb_y']}, {self.drone4_location['uwb_z']}), Orientation: {self.drone4_location['th']}, Status: {self.drone4_location['status']}")
        self.get_logger().info(f"Location Drone 5: ({self.drone5_location['x']}, {self.drone5_location['y']}, {self.drone5_location['z']}), UWB: ({self.drone5_location['uwb_x']}, {self.drone5_location['uwb_y']}, {self.drone5_location['uwb_z']}), Orientation: {self.drone5_location['th']}, Status: {self.drone5_location['status']}")

    def run(self):
        self.log_data()
        rclpy.spin_once(self, timeout_sec=5)
        self.call_tello_action('takeoff')
        self.log_data()
        time.sleep(self.altitude)
        self.print_info()
        rclpy.spin_once(self, timeout_sec=5)
        self.fly_forward_and_turn()
        self.log_data()
        time.sleep(self.altitude)
        self.print_info()
        self.call_tello_action('land')
        self.log_data()
        rclpy.spin_once(self, timeout_sec=5)
        self.print_info()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(self.drone1_location['x_history'], self.drone1_location['y_history'], self.drone1_location['z_history'], label="Copter 1 Position (Actual)", color="green")
        ax.plot(self.drone1_location['uwb_x_history'], self.drone1_location['uwb_y_history'], self.drone1_location['uwb_z_history'], label="Copter 1 Position (Estimation)", color="green", linestyle='dashed')
        ax.quiver(self.drone1_location['x_history'], self.drone1_location['y_history'], self.drone1_location['z_history'], 0, 0, self.drone1_location['th_history'], length=0.1, normalize=True, color="green", label="Copter 1 Yaw Orientation")
        
        ax.plot(self.drone2_location['x_history'], self.drone2_location['y_history'], self.drone2_location['z_history'], label="Copter 2 Position (Actual)", color="blue")
        ax.plot(self.drone2_location['uwb_x_history'], self.drone2_location['uwb_y_history'], self.drone2_location['uwb_z_history'], label="Copter 2 Position (Estimation)", color="blue", linestyle='dashed')
        ax.quiver(self.drone2_location['x_history'], self.drone2_location['y_history'], self.drone2_location['z_history'], 0, 0, self.drone2_location['th_history'], length=0.1, normalize=True, color="blue", label="Copter 2 Yaw Orientation")
        
        ax.plot(self.drone3_location['x_history'], self.drone3_location['y_history'], self.drone3_location['z_history'], label="Copter 3 Position (Actual)", color="red")
        ax.plot(self.drone3_location['uwb_x_history'], self.drone3_location['uwb_y_history'], self.drone3_location['uwb_z_history'], label="Copter 3 Position (Estimation)", color="red", linestyle='dashed')
        ax.quiver(self.drone3_location['x_history'], self.drone3_location['y_history'], self.drone3_location['z_history'], 0, 0, self.drone3_location['th_history'], length=0.1, normalize=True, color="red", label="Copter 3 Yaw Orientation")
          
        ax.plot(self.drone4_location['x_history'], self.drone4_location['y_history'], self.drone4_location['z_history'], label="Copter 4 Position (Actual)", color="purple")
        ax.plot(self.drone4_location['uwb_x_history'], self.drone4_location['uwb_y_history'], self.drone4_location['uwb_z_history'], label="Copter 4 Position (Estimation)", color="purple", linestyle='dashed')
        ax.quiver(self.drone4_location['x_history'], self.drone4_location['y_history'], self.drone4_location['z_history'], 0, 0, self.drone4_location['th_history'], length=0.1, normalize=True, color="purple", label="Copter 4 Yaw Orientation")
        
        ax.plot(self.drone5_location['x_history'], self.drone5_location['y_history'], self.drone5_location['z_history'], label="Copter 5 Position (Actual)", color="orange")
        ax.plot(self.drone5_location['uwb_x_history'], self.drone5_location['uwb_y_history'], self.drone5_location['uwb_z_history'], label="Copter 5 Position (Estimation)", color="orange", linestyle='dashed')
        ax.quiver(self.drone5_location['x_history'], self.drone5_location['y_history'], self.drone5_location['z_history'], 0, 0, self.drone5_location['th_history'], length=0.1, normalize=True, color="orange", label="Copter 5 Yaw Orientation")
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
