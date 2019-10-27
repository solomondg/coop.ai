import math
from dataclasses import dataclass
from typing import List
from random import random, shuffle
import numpy as np
import numpy.linalg
import networkx as nx
import networkx.algorithms
from enum import Enum

import carla

import zmq

from apis.Messages import Request
from controllers.drive_controller import DriveController
from mathutil.Pose2d import Pose2d
from mathutil.Translation2d import Translation2d
from mathutil.Rotation2d import Rotation2d
from mathutil.Types import carlaV3Dot
from services.MeshNode import MeshNode, defaultPortRange


@dataclass
class AgentRepresentation:
    """
    Represents a known or connected agent
    """
    port: int
    ssid: str

    def getGraph(self):
        pass  # TODO

    # Verify SSID against known - see if object has changed
    def verify(self) -> bool:
        return MeshNode.call(self.port, Request(function='ssid')).response != self.ssid

    def __key(self):
        return (self.port, self.ssid)

    def __hash__(self):
        return hash(self.__key())

    @staticmethod
    def fromAgent(a):
        return AgentRepresentation(port=a.portNumber, ssid=a.ssid)


MAX_CONNECTED_AGENTS = 3
# DIST_THRESHHOLD = 0.0002777778  # 1 second, or about 80 feet
# DIST_THRESHHOLD = 4
DIST_THRESHHOLD = 0.01666667
DIST_THRESHHOLD = 10000
MAX_GRAPH_DEPTH = 3

VEL_P_GAIN_ACC = 0.1
VEL_P_GAIN_BRK = 0.02

DIST_P_GAIN = 5
DIST_D_GAIN = 10


class AgentDrivingMode(Enum):
    ASSHOLE = -1
    MERGING_REQ = 0
    MERGING_ACT = 1
    DRIVING_HIGHWAY = 2
    DRIVING_CITY = 3
    TURNING = 4
    IDLE = 5


class AgentDrivingBehavior(Enum):
    FOLLOW_WAYPOINTS = 0
    MAINTAIN_DISTANCE = 1
    MERGING = 2
    WAITING = 3


class Agent(MeshNode):
    """
    Main agent microservice
    """
    connected_agents: List[AgentRepresentation] = []
    directly_connected: List[AgentRepresentation] = []
    found_agents: List[AgentRepresentation] = []
    gpscoords: np.ndarray
    graph: nx.Graph

    drivingMode: AgentDrivingMode
    drivingBehavior: AgentDrivingBehavior
    driveController: DriveController
    followTarget: AgentRepresentation
    followDistance: float = 8.0  # 3 meter

    PD_lastError: float = None

    velocityReference: float = 0.0

    waypointFollowSpeed: float = 50.0

    followAxis: Rotation2d

    def __init__(self, ssid: str = None, name: str = None, port: int = None, port_range: tuple = None):
        super().__init__(ssid=ssid, name=name, port=port,
                         port_range=port_range)  # Bind to port and start RPC/dispatch daemon

        self.gpscoords = np.asarray([0, 0])
        self.vehiclePose = Pose2d()
        self.dispatchTable['get_pose'] = self._getCoords
        self.dispatchTable['set_pose'] = self._setCoords
        # self.dispatchTable['get_connected'] = self._getCoords
        self.dispatchTable['get_graph_recursive'] = self.getGraph_Recursive
        self.dispatchTable['num_connections'] = lambda: len(self.directly_connected)
        self.dispatchTable['route_intra_graph_call'] = self.routeGraphRequest
        self.dispatchTable['intra_graph_call'] = self.execGraphRequest
        self.dispatchTable['ssid_lookup'] = self.getRepForSSID

        self.dispatchTable['negotiatePriority'] = self.negotiatePriority

        self.dispatchTable['connect_carla'] = self.connect_carla
        self.dispatchTable['spawn_vehicle'] = self.spawn_vehicle
        self.dispatchTable['drive_vehicle'] = self.drive_vehicle

        self.dispatchTable['tick'] = self._frameUpdate

        # ASSHOLE, MERGE_REQ, MERGE_ACT, DRIVING_HIGHWAY, DRIVING_CITY, TURNING, IDLE
        self.dispatchTable['set_driving_mode'] = self._setDrivingMode
        self.dispatchTable['get_driving_mode'] = lambda: self.drivingMode

        self.dispatchTable['set_follow_target'] = self._setFollowTarget
        self.dispatchTable['get_follow_target'] = lambda: self.followTarget
        self.dispatchTable['set_follow_distance'] = self._setFollowDistance
        self.dispatchTable['get_follow_distance'] = lambda: self.followDistance

        # FOLLOW_WAYPOINTS,
        self.dispatchTable['set_driving_behavior'] = self._setDrivingBehavior
        self.dispatchTable['get_driving_behavior'] = lambda: self.drivingBehavior

        self.dispatchTable['set_waypoint_follow_speed'] = lambda: self.drivingBehavior
        self.dispatchTable['get_waypoint_follow_speed'] = lambda: self.drivingBehavior

        self.dispatchTable['set_waypoints'] = self._setWaypoints()

        self.dispatchTable['get_throttle_brake'] = lambda: self.drivingBehavior
        self.dispatchTable['get_wheel'] = lambda: self.drivingBehavior

        self.dispatchTable['set_follow_axis'] = lambda: self.followAxis
        self.dispatchTable['get_follow_axis'] = self._setFollowAxis

        self.graph = nx.Graph()
        self.graph.add_node(AgentRepresentation.fromAgent(self))  # Add this

        self.carla_client = None
        self.carla_world = None
        self.carla_vehicle = None

        self.drivingMode = AgentDrivingMode.IDLE
        self.drivingBehavior = AgentDrivingBehavior.WAITING
        self.driveController = DriveController(2.5)

    def isAgentNode(self) -> bool:
        return True

    def _getCoords(self) -> np.ndarray:
        return self.pose

    def _setCoords(self, newCoords: Pose2d):
        self.gpscoords = Pose2d

    def _getDistance(self, otherCoords: np.ndarray) -> float:
        return np.linalg.norm(self.gpscoords - otherCoords)

    def findAgents(self):
        """
        :return: All agents in SSID scan range, sorted by distance
        """
        known_agents = []
        for port in range(defaultPortRange[0], defaultPortRange[1]):
            if port == self.portNumber:
                continue
            try:
                # print(f"Trying port {port}... ", end='')
                ssid = MeshNode.call(port, Request('ssid')).response
            except zmq.error.Again:
                # print(f"no agent found")
                pass
            else:
                agentCoords: np.ndarray = MeshNode.call(port, Request('get_coords')).response
                dist = self._getDistance(agentCoords)
                # print(f"agent found with coordinates {agentCoords} and distance {dist}.")
                if dist <= DIST_THRESHHOLD:
                    known_agents.append(AgentRepresentation(port=port, ssid=ssid))

        return sorted(known_agents, key= \
            lambda a: np.linalg.norm(MeshNode.call(a.port, Request('get_coords')).response - self.gpscoords))

        # To build graph:

    # Send over all previously traversed nodes
    # Ask for graph
    # If all connected nodes are already traversed or depth is max, return self (node)
    # Else, ask for graph from each non-already-traversed nodes, create edge from self (node) to them, return self (graph)

    def getGraph_Recursive(self, traversedAgents: List[AgentRepresentation]):

        print(f"gg_r called on {self.ssid}")

        if len(self.found_agents) == 0:
            self.found_agents = self.findAgents()

        agents = [agent for agent in self._semiShuffle(self.found_agents) if agent not in traversedAgents]

        # agents = [agent for agent in agents if len(self.graph.neighbors(agent)) < MAX_CONNECTED_AGENTS]

        print(f"untraversed agents: {[i.ssid for i in agents]}")

        if AgentRepresentation.fromAgent(self) not in traversedAgents:
            traversedAgents.append(AgentRepresentation.fromAgent(self))
        if len(agents) > 0:
            for agent in agents[0:min(len(agents), MAX_CONNECTED_AGENTS)]:
                otherrep = agent
                selfrep = AgentRepresentation.fromAgent(self)
                if MeshNode.call(otherrep.port, Request('num_connections')).response >= MAX_CONNECTED_AGENTS:
                    continue
                (otherGraph, additionallyTraversed) = MeshNode.call(otherrep.port, Request('get_graph_recursive',
                                                                                           args=[traversedAgents],
                                                                                           longRunning=True)).response
                traversedAgents.extend(additionallyTraversed)
                traversedAgents = list(set(traversedAgents))

                newGraph = nx.compose(otherGraph, self.graph)
                self.graph = newGraph.copy()
                # print(f"New graph: {otherGraph}")
                # self.graph.add_nodes_from(otherGraph)
                # self.graph.add_edges_from(otherGraph)
                self.graph.add_edge(selfrep, otherrep)
                self.directly_connected.append(otherrep)

        print(f"Number of connections: {len(list(self.graph.neighbors(AgentRepresentation.fromAgent(self))))}")
        if len(list(self.graph.neighbors(AgentRepresentation.fromAgent(self)))) < MAX_CONNECTED_AGENTS:
            # If there are no unconnected agents, let's add the closest ones that
            # A. we don't have any connections with
            # B. aren't at max connections
            nodes = list(self.graph.nodes())
            nodes.remove(AgentRepresentation.fromAgent(self))
            agents = []

            for agent in nodes:
                if len(list(self.graph.neighbors(agent))) < MAX_CONNECTED_AGENTS:
                    if not self.graph.has_edge(AgentRepresentation.fromAgent(self), agent):
                        agents.append(agent)
            agents2 = [a for a in self._semiShuffle(self.found_agents) if a in agents]
            agents = agents2
            for agent in agents[0: min(
                    MAX_CONNECTED_AGENTS - len(list(self.graph.neighbors(AgentRepresentation.fromAgent(self)))),
                    len(agents))]:
                print(f"Forming additional connection between {self.ssid} and {agent.ssid}")
                self.graph.add_edge(agent, AgentRepresentation.fromAgent(self))
                self.directly_connected.append(agent)

                # print(f"Returning traversed: {traversedAgents}")
        assert [len(list(self.graph.neighbors(i))) <= MAX_CONNECTED_AGENTS for i in self.graph.nodes()]
        return (self.graph, traversedAgents)

    # Query for all connected SSIDs -> ping each SSID and repeat the process
    # Godmode because you're overriding the mesh topology and just directly pinging (cheating, but easier)
    def getGraph_Godmode(self):
        return

    @staticmethod
    def _semiShuffle(l: List, probability: float = 0.3) -> List:
        l2 = l.copy()
        for i in range(len(l2) - 1):
            if random() < probability:
                l2[i], l2[i + 1] = l2[i + 1], l2[i]
        return l2

    def connect_carla(self, ip: str = 'localhost', port: int = 2000):
        self.carla_client = carla.Client(ip, port)
        self.carla_world = self.carla_client.get_world()

    def spawn_vehicle(self, x, y, z, yaw, template_filter='*.tesla.*'):
        blueprint = self.carla_world.get_blueprint_library().filter(template_filter)[0]
        spawnpoint = carla.Tranform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
        self.carla_vehicle = self.carla_world.spawn_actor(blueprint, spawnpoint)

    def drive_vehicle(self, throttle: float, brake: float, wheel: float):
        control = carla.VehicleControl(throttle, brake, wheel, False, False, False, 0)
        self.carla_vehicle.apply_control(control)

    def _setDrivingMode(self, mode: AgentDrivingMode):
        self.drivingMode = mode

    def _routeTo(self, node: AgentRepresentation):
        return nx.algorithms.shortest_path(self.graph, AgentRepresentation.fromAgent(self), node)

    def routeGraphRequest(self, req: Request, path: List[AgentRepresentation]):
        if len(path) == 1:
            return self.dispatch(req).response
        else:
            path.pop(0)
            return MeshNode.call(path[0].port, req=Request("route_intra_graph_call", [req, path])).response

    def execGraphRequest(self, req: Request, endpoint: AgentRepresentation):
        path = self._routeTo(endpoint)
        req = Request("route_intra_graph_call", [req, path])
        return self.dispatch(req).response

    def getRepForSSID(self, ssid: str):
        return [i for i in self.graph.nodes if i.ssid == ssid][0]

    def negotiatePriority(self, other: AgentRepresentation):
        ourMode = self.drivingMode
        theirMode = self.dispatch(Request('intra_graph_call', args=[Request('get_driving_mode'), other])).response
        if ourMode > theirMode:
            self._determineYieldAction(ourMode, theirMode)

    def _determineYieldAction(self, ourMode: AgentDrivingMode, theirMode: AgentDrivingMode):
        pass

    def _frameUpdate(self):
        vel = self._getCarForwardVelocity()
        throttle, brake = self._getThrottleAndBrake(vel)
        wheel = 0.0
        self.drive_vehicle(throttle, brake, wheel)

    def _setFollowTarget(self, target: AgentRepresentation):
        self.followTarget = target

    def _setFollowDistance(self, dist: float):
        self.followDistance = dist

    def _setDrivingBehavior(self, behavior: AgentDrivingBehavior):
        self.drivingBehavior = behavior

    def _setVelocityRef(self, vref: float):
        self.velocityReference = vref

    def _setWaypointSpeed(self, vref: float):
        self.waypointFollowSpeed = vref

    def _getThrottleAndBrake(self, vel=None):
        if vel is None:
            self._getCarForwardVelocity()
        error = vel - self.velocityReference
        if error > 0:  # actual speed > desired, need to brake
            throttle = 0.0
            brake = VEL_P_GAIN_BRK * math.fabs(error)
        else:  # actual speed < desired, need to accelerate
            throttle = VEL_P_GAIN_ACC * math.fabs(error)
            brake = 0.0
        return (throttle, brake)

    def _getCarForwardVelocity(self):
        vel: carla.Vector3D = self.carla_vehicle.get_velocity()
        forwardVector: carla.Vector3D = self.carla_vehicle.get_transform().rotation.get_forward_vector()
        return carlaV3Dot(vel, forwardVector)

    def _distanceToFollowTarget(self) -> Translation2d:
        our_pose = self.vehiclePose
        their_pose: Pose2d = self.dispatch(Request('intra_graph_call',
                                                   args=[Request('get_coords'), self.followTarget])).response
        pose_difference = their_pose.translation - our_pose.translation
        diff_along_axis: float = pose_difference.position.dot(self.followAxis)
        return diff_along_axis

    def _runFollowPDLoop(self, distance=None):
        if distance is None:
            distance = self._distanceToFollowTarget()
        currentError = distance - self.followDistance
        if self.PD_lastError is None:
            self.PD_lastError = currentError
        dError = currentError - self.PD_lastError
        pTerm = DIST_P_GAIN * currentError
        dTerm = DIST_D_GAIN * dError
        return pTerm + dTerm

    def _setFollowAxis(self, axis: Rotation2d):
        self.followAxis = axis

    def _setWaypoints(self):
        pass


def test_findSSIDs():
    N = 10
    objs = []
    for i in range(N):
        a = Agent(ssid=f'AGENT-{str(i)}')
        MeshNode.call(a.portNumber, Request("set_coords", args=[np.asarray([random(), random()])]))
        objs.append(a)

    print(objs[0].findAgents())


def test_findNodes():
    N = 4
    objs = []
    coordDict = {}
    for i in range(N):
        a = Agent(ssid=f'AGENT-{str(i)}')
        pos = np.asarray([random(), random()])
        coordDict[AgentRepresentation.fromAgent(a)] = pos
        MeshNode.call(a.portNumber, Request("set_coords", args=[pos]))
        objs.append(a)

    (graph, traversed) = MeshNode.call(objs[0].portNumber,
                                       Request('get_graph_recursive', args=[[]], longRunning=True)).response

    print(f"Traversed total: {[i.ssid for i in traversed]}")

    import matplotlib.pyplot as plt
    # nx.draw_networkx(graph, pos=coordDict,
    #                 labels={key: key.ssid for key in [AgentRepresentation.fromAgent(a) for a in objs]})
    nx.draw_networkx(graph, pos=coordDict,
                     labels={key: key.ssid for key in [AgentRepresentation.fromAgent(a) for a in objs]})
    # nx.draw_random(graph)
    # nx.draw_circular(graph)
    # nx.draw_spectral(graph)
    # nx.draw_planar(graph)
    plt.show()


def test_pathing():
    N = 3
    objs = []
    coordDict = {}
    for i in range(N):
        a = Agent(ssid=f'AGENT-{str(i)}')
        pos = np.asarray([random(), random()])
        coordDict[AgentRepresentation.fromAgent(a)] = pos
        MeshNode.call(a.portNumber, Request("set_coords", args=[pos]))
        objs.append(a)

    (graph, traversed) = MeshNode.call(objs[0].portNumber,
                                       Request('get_graph_recursive', args=[[]], longRunning=True)).response

    # route = objs[0]._routeTo(objs[0].found_agents[3])
    # print(objs[0].found_agents[3], route)
    targetAgent = MeshNode.call(objs[0].portNumber, Request('ssid_lookup', args=['AGENT-1'])).response
    targetAgentCoords = MeshNode.call(objs[0].portNumber,
                                      Request('intra_graph_call', args=[Request('get_coords'), targetAgent])).response
    print(targetAgent, targetAgentCoords)
    print(objs[0].negotiatePriority(targetAgent))


if __name__ == "__main__":
    # test_findSSIDs()
    test_findNodes()
    # test_pathing()
