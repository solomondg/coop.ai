from abc import ABC, abstractmethod
import os, binascii
import zmq
import time
from threading import Thread

from typing import Dict, Callable

from apis.Messages import Request, Response, Message, RequestStatus, RPCErrorType

defaultPortRange = (8000, 8100)


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

    def __init__(self, ssid: str = None, name: str = None,
                 port: int = None, port_range: tuple = None):
        self.ssid = binascii.b2a_hex(os.urandom(15)).decode() if ssid is None else ssid
        self.name = self.ssid if name is None else name
        self.dispatchTable = {}
        self.dispatchTable['ssid'] = self.getSSID
        self.dispatchTable['list_functions'] = self.getFunctions
        self.dispatchTable['is_coopai'] = self.isAgentNode
        port_range = defaultPortRange if port_range is None else port_range

        self.bind(port=port, port_range=port_range)
        self.start()

        super().__init__()

    def bind(self, port: int = None, port_range: tuple = defaultPortRange):
        self.socket = zmq.Context().socket(zmq.REP)

        if port is None:  # time to auto find port
            found = False
            for searchPoint in range(port_range[0], port_range[1]):
                try:
                    self.socket.bind('tcp://*:{}'.format(searchPoint))
                except zmq.ZMQError:
                    pass
                else:
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

        print(f"Agent {self.name} connected to socket {self.portNumber}")

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
        if req.function in ['get_graph_recursive', 'intra_graph_call', 'route_intra_graph_call',
                            'tick']:
            req.longRunning = True
        s = zmq.Context().socket(zmq.REQ)
        if not req.longRunning:
            s.setsockopt(zmq.RCVTIMEO, 10)
            s.setsockopt(zmq.SNDTIMEO, 10)
            s.setsockopt(zmq.LINGER, 0)
        else:
            s.setsockopt(zmq.RCVTIMEO, -1)
            s.setsockopt(zmq.SNDTIMEO, -1)
            s.setsockopt(zmq.LINGER, 1)
        s.connect("tcp://localhost:{}".format(port))
        s.send(req.serialize())
        try:
            recv = s.recv()
        except zmq.error.Again as a:
            raise a
        finally:
            s.close()
        resp = Response.load(recv)
        return resp
