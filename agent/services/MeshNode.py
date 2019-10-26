from abc import ABC, abstractmethod
import os, binascii
import zmq
import time
from threading import Thread

from typing import Dict, Callable

from apis.Messages import Request, Response, Message, RequestStatus, RPCErrorType

defaultPortRange = (8000, 9000)


class MeshNode(ABC):
    ssid: str
    portNumber: int
    socket: zmq.Socket
    name: str
    dispatchTable: Dict[str, Callable[[Request], Response]]

    running: bool
    dispatchThread: Thread

    socket_opened = False
    thread_started = False

    def __init__(self, ssid=binascii.b2a_hex(os.urandom(15)).decode(), name=None):
        self.ssid = ssid
        self.name = ssid if name is None else name
        self.dispatchTable = {}
        self.dispatchTable['ssid'] = self.getSSID
        self.dispatchTable['list_functions'] = self.getFunctions
        self.dispatchTable['is_coopai'] = self.isAgentNode

        super().__init__()

    def bind(self, port: int = None, port_range: tuple = defaultPortRange):
        self.socket = zmq.Context().socket(zmq.REP)

        if port is None:  # time to auto find port
            found = False
            for searchPoint in range(port_range[0], port_range[1]):
                print(f"Trying {searchPoint}...")
                try:
                    self.socket.bind('tcp://*:{}'.format(searchPoint))
                except zmq.ZMQError:
                    print("Socket in use")
                else:
                    print("Socket available!")
                    found = True
                    break
            if not found: raise Exception("Open port not found!")

        else:
            self.portNumber = port
            try:
                self.socket.bind('tcp://*:{}'.format(self.portNumber))
            except Exception:
                raise Exception("Open port not found!")

        self.portNumber = port if port is not None else searchPoint

        self.socket_opened = True

    def start(self):
        self.running = True
        self.thread_started = True
        self.dispatchThread = Thread(target=self._thread, name=self.name, args=())
        self.dispatchThread.daemon = True
        self.dispatchThread.start()

    def stop(self):
        self.running = False
        self.dispatchThread.join()

    def _thread(self):
        while self.running:
            request_b = self.socket.recv()
            request = Request.load(request_b)
            response = self.dispatch(request)
            response_b = response.serialize()
            self.socket.send(response_b)

    def getSSID(self) -> str:
        return self.ssid

    def getFunctions(self) -> str:
        return list(self.dispatchTable.keys())

    @abstractmethod
    def isAgentNode(self) -> bool:
        pass

    def dispatch(self, req: Request) -> Response:
        fcnName = req.function
        fcnArgs = req.args
        if req.function in self.dispatchTable.keys():
            return Response(status=RequestStatus.SUCCESS, response=self.dispatchTable[fcnName](*fcnArgs))
        else:
            return Response(status=RequestStatus.FAILURE, response=RPCErrorType.FUNCTION_NOT_FOUND)

    def __del__(self):
        if self.thread_started:
            self.stop()
        if self.socket_opened:
            self.socket.close()

    @staticmethod
    def call(port: int, req: Request) -> Response:
        s = zmq.Context().socket(zmq.REQ)
        s.setsockopt(zmq.RCVTIMEO, 100)
        s.setsockopt(zmq.SNDTIMEO, 100)
        s.connect("tcp://localhost:{}".format(port))
        s.send(req.serialize())
        resp = Response.load(s.recv())
        return resp
