#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from tello_msgs.srv import TelloAction
import time
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

class TelloCommander(Node):
    def __init__(self):
        super().__init__('tello_commander')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.service_client = self.create_client(TelloAction, '/drone1/tello_action')
        self.odom_subscriber = self.create_subscription(Odometry, '/drone1/odom', self.odom_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/drone1/pose', self.pose_callback, 10)
        self.speed = 0.5
        self.turn = 1.5
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

    def calculate_uwb(self):
        A = (5, -4)
        B = (1, -2)
        C = (5, 4)
        self.uwb_x = math.sqrt((self.x - A[0])**2 + (self.y - A[1])**2)
        self.uwb_y = math.sqrt((self.x - B[0])**2 + (self.y - B[1])**2)
        self.uwb_z = math.sqrt((self.x - C[0])**2 + (self.y - C[1])**2)
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
        # while not self.service_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for service...')
        request = TelloAction.Request()
        request.cmd = command
        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result()}')
        else:
            self.get_logger().warning('Service call failed: %r' % (future.exception(),))

    def fly_forward_and_turn(self):
        self.send_cmd_vel(linear_x=1)
        self.get_logger().info('Flying forward...')
        rclpy.spin_once(self, timeout_sec=2)
        self.send_cmd_vel(angular_z=1)
        self.get_logger().info('Turning...')
        rclpy.spin_once(self, timeout_sec=2)
        self.send_cmd_vel(linear_x=-1, angular_z=-1)
        self.get_logger().info('Flying backward...')
        rclpy.spin_once(self, timeout_sec=2)
        self.send_cmd_vel()
        self.get_logger().info('Stopping...')
        rclpy.spin_once(self, timeout_sec=2)

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
        self.call_tello_action('takeoff')
        time.sleep(4)
        self.print_info()
        self.fly_forward_and_turn()
        time.sleep(4)
        self.print_info()
        self.call_tello_action('land')
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
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    commander = TelloCommander()
    commander.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
