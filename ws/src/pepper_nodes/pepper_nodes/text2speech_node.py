#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pepper_interfaces.srv import Text2Speech 
from pepper_nodes import PepperNode
from pepper_nodes.utils import Session

class Text2SpeechNode(PepperNode):

    def __init__(self):
        super().__init__('text2speech_node')
        self.session = Session(self.ip, self.port)
        self.tts = self.session.get_service("ALTextToSpeech")

        self.tts_service = self.create_service(Text2Speech, 'tts', self.say_callback)

        self.get_logger().info("Text2SpeechNode initialized")

    def say_callback(self, request, response):
        try:
            self.tts.say(request.speech)
        except Exception as e:
            self.get_logger().warn(f"TTS failed, retrying: {e}")
            self.session.reconnect()
            self.tts = self.session.get_service("ALTextToSpeech")
            self.tts.say(request.speech)
        response.result = "ACK"
        return response

def main():

    rclpy.init()
    node = Text2SpeechNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
