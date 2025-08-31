#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pepper_interfaces.srv import WakeUp, Rest
from pepper_nodes import PepperNode
from pepper_nodes.utils import Session

class WakeUpNode(PepperNode):

    def __init__(self):
        super().__init__('wakeup_node')
        self.session = Session(self.ip, self.port)
        self.motion_proxy = self.session.get_service("ALMotion")
        self.posture_proxy = self.session.get_service("ALRobotPosture")

        self.wakeup_service = self.create_service(WakeUp, 'wakeup', self.wakeup_callback)
        self.rest_service = self.create_service(Rest, 'rest', self.rest_callback)

        self.get_logger().info("WakeUpNode initialized")
        self.wakeup()
        self.stand()

    def rest_callback(self, request, response):
        try:
            self.motion_proxy.rest()
        except Exception as e:
            self.get_logger().warn(f"Rest failed, retrying: {e}")
            self.motion_proxy = self.session.get_service("ALMotion")
            self.motion_proxy.rest()
        response.result = "ACK"
        return response

    def wakeup_callback(self, request, response):
        try:
            self.motion_proxy.wakeUp()
            self.stand()
        except Exception as e:
            self.get_logger().warn(f"WakeUp failed, retrying: {e}")
            self.motion_proxy = self.session.get_service("ALMotion")
            self.posture_proxy = self.session.get_service("ALRobotPosture")
            self.motion_proxy.wakeUp()
            self.stand()
        response.result = "ACK"
        return response

    def stand(self):
        self.posture_proxy.goToPosture("StandInit", 0.5)

def main():

    rclpy.init()
    node = WakeUpNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
