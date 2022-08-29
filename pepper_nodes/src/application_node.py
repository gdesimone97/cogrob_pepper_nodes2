#!/usr/bin/python3

import rospy
from utils import Application

if __name__ == "__main__":
    ip = rospy.get_param("ip")
    port = rospy.get_param("port", "9559")
    NODE_NAME = "application_node"
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"{NODE_NAME} started")
    app = Application(ip, port)
    app._application.start()
    app._application.run()