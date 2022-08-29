#!/usr/bin/python3
import qi
import sys

class Session:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self._session = qi.Session()
        self._connect()

    def _connect(self):
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("Can't connect to Naoqi at ip \"" + self.ip + "\" on port " + str(self.port) + ".\n"
                                                                                                 "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)

    def reconnect(self):
        self._connect()
        return self.session

    @property
    def session(self):
        return self._session

    def get_service(self, service_name: str):
        return self.session.service(service_name)