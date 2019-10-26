from dataclasses import dataclass
from typing import List
import numpy as np
import numpy.linalg

import zmq

from apis.Messages import Request
from services.MeshNode import MeshNode, defaultPortRange


@dataclass
class AgentRepresentation:
    """
    Represents a known or connected agent
    """
    port: int
    ssid: str
    connected: bool

    def getGraph(self):
        pass  # TODO

    # Verify SSID against known - see if object has changed
    def verify(self) -> bool:
        return MeshNode.call(self.port, Request(function='ssid')).response != self.ssid


MAX_CONNECTED_AGENTS = 7
DIST_THRESHHOLD = 0.0002777778 # 1 second, or about 80 feet


class Agent(MeshNode):
    """
    Main agent microservice
    """
    connected_agents: List[AgentRepresentation] = []

    gpscoords: np.ndarray

    def __init__(self):
        self.gpscoords = np.asarray([0, 0])
        self.dispatchTable['get_coords'] = self._getCoords

        super().__init__()

    def isAgentNode(self) -> bool:
        return True

    def _getCoords(self) -> tuple[float, float]:
        return self.gpscoords

    def _getDistance(self, otherCoords: np.ndarray) -> float:
        return np.linalg.norm(self.gpscoords-otherCoords)

    def findAgents(self):
        known_agents = []
        for port in range(defaultPortRange[0], defaultPortRange[1]):
            try:
                print(f"Trying port {port}... ",end='')
                ssid = MeshNode.call(port, Request('ssid')).response
            except zmq.error.Again:
                print(f"no agent found")
            else:
                agentCoords: np.ndarray = MeshNode.call(i, Request('get_coords')).response
                dist = self._getDistance(agentCoords)
                print(f"agent found with coordinates {agentCoords} and distance {dist}.")
                if dist <= DIST_THRESHHOLD:
                    known_agents.append(AgentRepresentation(port=port, ssid=ssid, connected=False))

