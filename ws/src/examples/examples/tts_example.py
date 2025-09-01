#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from time import sleep
from pepper_interfaces.srv import Text2Speech

class TTSHandler(Node):

    def __init__(self):
        super().__init__('tts_node_example')
        self.cli = self.create_client(Text2Speech, '/tts')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /tts service...')

    def call(self, text: str, language="", volume=Text2Speech.Request.NO_SET):
        request = Text2Speech.Request()
        request.speech = text
        request.language = language
        request.volume = volume

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Response: {future.result().ack}")
        else:
            self.get_logger().error("Service call failed")

def main():
    rclpy.init()
    node = TTSHandler()

    text = "Hi! I'm Pepper"
    node.call(text, language=Text2Speech.Request.ENGLISH, volume=70)
    sleep(2.0)
    text = "Ciao! Sono Pepper"
    node.call(text, language=Text2Speech.Request.ITALIAN)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
