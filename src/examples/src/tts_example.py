#!/usr/bin/python3

import rospy
from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse

class Handler:
    def __init__(self):
        self.tts = rospy.ServiceProxy("/tts", Text2Speech)

    def call(self, text: str):
        msg = Text2SpeechRequest()
        msg.speech = text
        resp = self.tts(text)
        rospy.loginfo(resp.ack)

if __name__ == "__main__":
    NODE_NAME = "tts_node_example"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    text = "Hi! I'm Pepper"
    handler.call(text)
