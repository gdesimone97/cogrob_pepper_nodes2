#!/usr/bin/python3

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

class Session:
    def __init__(self):
        self.app = Application()

    def reconnect(self):
        self.app.reconnect()
        return self.session

    @property
    def session(self):
        return self.app._application.session

    def get_service(self, service_name: str):
        return self.session.service(service_name)