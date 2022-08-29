#!/usr/bin/python

import rospy
import qi

class SingletonMeta(type):

    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

class Application(metaclass=SingletonMeta):
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self._application = qi.Application(url=f"tcp://{self.ip}:{self.port}")

    def reconnect(self):
        self._application.stop()
        self._application = qi.Application(url=f"tcp://{self.ip}:{self.port}")
        self._application.start()
        self._application.run()

if __name__ == "__main__":
    ip = rospy.get_param("ip")
    port = rospy.get_param("port", "9559")
    NODE_NAME = "application_node"
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"{NODE_NAME} started")
    app = Application(ip, port)
    app._application.start()
    app._application.run()