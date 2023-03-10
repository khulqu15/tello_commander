#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tello_msgs.srv import TelloAction
import time

class TelloCommander(Node):
    def __init__(self):
        super().__init__('tello_commander')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.service_client = self.create_client(TelloAction, '/drone1/tello_action')
        self.speed = 0.5
        self.turn = 1.5
        self.pose_subscriber = self.create_subscription(PoseStamped, '/drone1/pose', self.pose_callback, 10)
        # self.x = 0.0
        # self.y = 0.0
        # self.z = 0.0
        # self.th = 0.0
        # self.status = 0.0

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.get_logger().info("Drone postion: x={x}, y={y}, z={z}")

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
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
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

    def run(self):
        self.call_tello_action('takeoff')
        time.sleep(4)
        self.fly_forward_and_turn()
        time.sleep(4)
        self.call_tello_action('land')

def main(args=None):
    rclpy.init(args=args)
    commander = TelloCommander()
    commander.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
