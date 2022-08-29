#!/usr/bin/env python

from application_node import SingletonMeta

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