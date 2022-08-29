#!/usr/bin/python3

from utils import Session
from optparse import OptionParser
from std_msgs.msg import Float32MultiArray
import rospy

class HeadMotionNode:

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.motion_proxy = self.session.get_service("ALMotion")

    def head_yaw(self, msg):
        try:
            self.motion_proxy.setAngles(["HeadYaw"], [msg.data[0]], msg.data[1])
        except:
            self.motion_proxy = self.motion_proxy = self.session.get_service("ALMotion")
            self.motion_proxy.setAngles(["HeadYaw"], [msg.data[0]], msg.data[1])

    def head_pitch(self, msg):
        try:
            self.motion_proxy.setAngles(["HeadPitch"], [msg.data[0]], msg.data[1])
        except:
            self.motion_proxy = self.motion_proxy = self.session.get_service("ALMotion")
            self.motion_proxy.setAngles(["HeadPitch"], [msg.data[0]], msg.data[1])

    def start(self):
        rospy.init_node("head_motion_node")
        rospy.Subscriber("/head_rotation/yaw", Float32MultiArray, self.head_yaw)
        rospy.Subscriber("/head_rotation/pitch", Float32MultiArray, self.head_pitch)
        rospy.spin()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = HeadMotionNode(options.ip, int(options.port))
        node.start()
    except rospy.ROSInterruptException:
        pass
