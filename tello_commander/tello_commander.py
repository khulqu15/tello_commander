#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction
import time

class TelloCommander(Node):
    def __init__(self):
        super().__init__('tello_commander')
        self.service_client = self.create_client(TelloAction, '/drone1/tello_action')

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

def main(args=None):
    rclpy.init(args=args)
    commander = TelloCommander()
    commander.call_tello_action('takeoff')
    time.sleep(4)
    commander.call_tello_action('landing')
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()