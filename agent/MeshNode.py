from abc import ABC, abstractmethod
import os, binascii
import zmq
from jsonrpcserver import method, dispatch
import time
from threading import Thread

from typing import Dict, Callable


class MeshNode(ABC):
    ssid: str
    portNumber: int
    socket: zmq.Socket
    name: str
    dispatchTable: Dict[str, Callable]

    def __init__(self, ssid=binascii.b2a_hex(os.urandom(15)).decode(), name=None):
        self.ssid = ssid
        self.name = ssid if name is None else name
        self.dispatch = {}
        self.dispatch['ssid'] = self.getSSID

        super().__init__()

    def bind(self, port: int = None, port_range: tuple = (8000, 9000)):
        self.socket = zmq.Context().socket(zmq.REP)

        if port is None:  # time to auto find port
            self.portNumber = 5000
            searchPoint: int = port_range[0]
            found = False
            # while (searchPoint < port_range[1]) and (not found):
        else:
            self.portNumber = port

        self.socket.bind('tcp://*:{}'.format(self.portNumber))

        while True:
            request = self.socket.recv().decode()
            response = dispatch(request)
            self.socket.send_string(str(response))

    def getSSID(self) -> str:
        return self.ssid

    @abstractmethod
    def dispatch(self, message: Message) -> Response:
        pass

    def __del__(self):
        self.socket.close()
