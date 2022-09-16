#!/usr/bin/python3

import rospy
from pepper_nodes.srv import LoadUrl, LoadUrlRequest, LoadUrlResponse

class Handler:
    def __init__(self):
        self.tablet_service = rospy.ServiceProxy("load_url", LoadUrl)

    def load_url(self, url):
        msg = LoadUrlRequest()
        msg.url = url
        self.tablet_service(msg)

if __name__ == "__main__":
    NODE_NAME = "table_node_example"
    rospy.init_node(NODE_NAME)
    handler = Handler()
    url = r"https://www.diem.unisa.it/"
    handler.load_url(url)
