#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pepper_interfaces.srv import ExecuteJS, LoadUrl
from pepper_nodes import PepperNode
from pepper_nodes.utils import Session

class TabletNode(PepperNode):

    def __init__(self):
        super().__init__('tablet_node')
        self.session = Session(self.ip, self.port)
        self.tablet_proxy = self.session.get_service("ALTabletService")
        self.tablet_proxy.resetTablet()

        self.execute_js_service = self.create_service(ExecuteJS, 'execute_js', self.execute_js_callback)
        self.load_url_service = self.create_service(LoadUrl, 'load_url', self.load_url_callback)

        self.get_logger().info("TabletNode initialized")

    def load_url_callback(self, request, response):
        try:
            self.tablet_proxy.showWebview(request.url)
        except Exception as e:
            self.get_logger().warn(f"Load URL failed, retrying: {e}")
            self.tablet_proxy = self.session.get_service("ALTabletService")
            self.tablet_proxy.showWebview(request.url)
        response.result = "ACK"
        return response

    def execute_js_callback(self, request, response):
        try:
            self.tablet_proxy.executeJS(request.js)
        except Exception as e:
            self.get_logger().warn(f"Execute JS failed, retrying: {e}")
            self.tablet_proxy = self.session.get_service("ALTabletService")
            self.tablet_proxy.executeJS(request.js)
        response.result = "ACK"
        return response

def main():
    rclpy.init()
    node = TabletNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()