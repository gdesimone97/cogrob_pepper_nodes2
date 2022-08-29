#!/usr/bin/env python
from utils import Session
from optparse import OptionParser
from pepper_nodes.srv import *
import rospy

class Text2SpeechNode:

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session()
        self.tts = self.session.get_service("ALTextToSpeech")
        

    def say(self, msg):
        try:
            self.tts.say(msg.speech)
        except:
            self.session.reconnect()
            self.tts = self.session.get_service("ALTextToSpeech")
            self.tts.say(msg.speech)
        return "ACK"
    
    def start(self):
        rospy.init_node("text2speech_node")
        rospy.Service('tts', Text2Speech, self.say)

        rospy.spin()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        ttsnode = Text2SpeechNode(options.ip, int(options.port))
        ttsnode.start()
    except rospy.ROSInterruptException:
        pass