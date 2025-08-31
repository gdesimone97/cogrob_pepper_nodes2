#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraShowNode(Node):

    def __init__(self):
        super().__init__('camera_show_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/in_rgb',
            self.callback,
            10
        )

        self.get_logger().info("CameraShowNode initialized")

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv2.imshow("Pepper Camera", image)
            cv2.waitKey(50)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main():
    rclpy.init()
    node = CameraShowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        import sys
        print(sys.exc_info())
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
