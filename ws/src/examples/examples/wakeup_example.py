#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pepper_interfaces.srv import WakeUp, Rest
from time import sleep

class WakeRestHandler(Node):

    def __init__(self):
        super().__init__('wake_rest_node_example')

        self.wakeup_client = self.create_client(WakeUp, 'wakeup')
        self.rest_client = self.create_client(Rest, 'rest')

        while not self.wakeup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for wakeup service...')
        while not self.rest_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for rest service...')

    def wakeup(self):
        request = WakeUp.Request()
        future = self.wakeup_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"WakeUp response: {future.result().ack}")
        else:
            self.get_logger().error("WakeUp service call failed")

    def rest(self):
        request = Rest.Request()
        future = self.rest_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"Rest response: {future.result().ack}")
        else:
            self.get_logger().error("Rest service call failed")

def main():
    rclpy.init()
    node = WakeRestHandler()

    SLEEP = 3.0

    node.get_logger().info("I am standing up")
    node.wakeup()

    node.get_logger().info(f"Waiting for {SLEEP} seconds")
    sleep(SLEEP)

    node.get_logger().info("I'm going to rest")
    node.rest()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
