#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
class HeadMotionPublisher(Node):

    def __init__(self):
        super().__init__('head_motion_publisher')
        self.head_motion_pitch_pub = self.create_publisher(Float32MultiArray, '/head_rotation/pitch', 10)
        self.head_motion_yaw_pub = self.create_publisher(Float32MultiArray, '/head_rotation/yaw', 10)

    def has_subscribers(self, publisher):
        if publisher.get_subscription_count() <= 0:
            self.get_logger().info("Waiting for subscribers for {}".format(publisher.topic_name))
            return False
        return True
    
    def move_head_yaw(self, angle, velocity=0.2):
        msg = Float32MultiArray()
        msg.data = [angle, velocity]
        self.get_logger().info(f"Moving head relative to yaw of {angle} radians")
        self.head_motion_yaw_pub.publish(msg)

    def move_head_pitch(self, angle, velocity=0.2):
        msg = Float32MultiArray()
        msg.data = [angle, velocity]
        self.get_logger().info(f"Moving head relative to pitch of {angle} radians")
        self.head_motion_pitch_pub.publish(msg)

def main():
    rclpy.init()
    VELOCITY = 0.15
    SLEEP_TIME_SEC = 3.0
    
    node = HeadMotionPublisher()
    
    while not node.has_subscribers(node.head_motion_yaw_pub):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.move_head_yaw(0.5, VELOCITY)
    
    while not node.has_subscribers(node.head_motion_yaw_pub):
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.spin_once(node, timeout_sec=SLEEP_TIME_SEC)
    node.move_head_yaw(0.0, VELOCITY)
    
    while not node.has_subscribers(node.head_motion_pitch_pub):
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.spin_once(node, timeout_sec=SLEEP_TIME_SEC)
    node.move_head_pitch(-0.35, VELOCITY)
    
    while not node.has_subscribers(node.head_motion_pitch_pub):
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.spin_once(node, timeout_sec=SLEEP_TIME_SEC)
    node.move_head_pitch(0.0, VELOCITY)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()