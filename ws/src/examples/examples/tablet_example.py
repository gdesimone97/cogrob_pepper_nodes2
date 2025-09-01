#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pepper_interfaces.srv import LoadUrl

class TabletHandler(Node):

    def __init__(self):
        super().__init__('tablet_node_example')
        self.cli = self.create_client(LoadUrl, 'load_url')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for load_url service...')

    def load_url(self, url):
        request = LoadUrl.Request()
        request.url = url

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Response: {future.result().ack}")
        else:
            self.get_logger().error("Service call failed")

def main():
    rclpy.init()
    node = TabletHandler()

    url = "https://www.diem.unisa.it/"
    node.load_url(url)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
