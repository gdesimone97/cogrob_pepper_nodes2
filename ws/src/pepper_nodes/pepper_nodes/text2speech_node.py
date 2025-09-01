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
        self.tts_proxy = self.session.get_service("ALTextToSpeech")
        self.audio_device_proxy = self.session.get_service("ALAudioDevice")
        self.tts_service = self.create_service(Text2Speech, 'tts', self.say_callback)
        self.get_logger().info("Text2SpeechNode initialized")

    def set_volume(self, volume: int):
        if volume == Text2Speech.Request.NO_SET:
            return
        if volume not in range(0, 101):
            self.get_logger().error("Volume not set for value {}. Must be between [0-100].".format(volume))
            return
        volume = int(volume)
        self.audio_device_proxy.setOutputVolume(volume)
        self.get_logger().info(f"Setting volume to {volume}")
    
    def set_language(self, language: str):
        if language == "":
            return
        if language not in [Text2Speech.Request.ENGLISH, Text2Speech.Request.ITALIAN]:
            self.get_logger().error(f"Language not set for value {language}. Must be one among {[Text2Speech.Request.ENGLISH, Text2Speech.Request.ITALIAN]}")
            return
        self.tts_proxy.setLanguage(language)
        self.get_logger().info(f"Language set to {language}")
    
    def say_callback(self, request: Text2Speech.Request, response: Text2Speech.Response):
        try:
            speech = request.speech
            self.set_volume(request.volume)
            self.set_language(request.language)
            self.tts_proxy.say(speech)
        except Exception as e:
            self.get_logger().warn(f"TTS failed, retrying: {e}")
            self.session.reconnect()
            self.tts_proxy = self.session.get_service("ALTextToSpeech")
            self.audio_device_proxy = self.session.get_service("ALAudioDevice")
            speech = request.speech
            self.set_volume(request.volume)
            self.set_language(request.language)
            self.tts_proxy.say(speech)
        response.ack = "ACK"
        return response

def main():

    rclpy.init()
    node = Text2SpeechNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        import sys
        print(sys.exc_info())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
