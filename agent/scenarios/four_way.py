from time import time, sleep

import carla

from mathutil.Rotation2d import Rotation2d
from services.Agent import Agent, AgentDrivingBehavior, AgentRepresentation, AgentDrivingMode
from services.MeshNode import MeshNode
from apis.Messages import Request
from mathutil.Translation2d import Translation2d

override_dt = 0.1


class fourway:
    def __init__(self, ip='localhost', port=2000):
        self.carla_client = carla.Client(ip, port)
        self.world = None
        self.car_a = None
        self.car_b = None
        self.spectators = []

    def setup(self):
        self.initialize_carla_world()
        self.place_spectator()
        self.clear_vehicles()
        self.spawn_vehicles()

    def initialize_carla_world(self):
        self.carla_client.set_timeout(10)
        self.world = self.carla_client.load_world('Town05')
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.01
        self.world.apply_settings(settings)

    def place_spectator(self, x=-43, y=3, z=45, pitch=-90, yaw=0):
        spectator = [i for i in self.world.get_actors() if i.type_id == 'spectator'][0]
        spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z),
                                                carla.Rotation(pitch=pitch, yaw=yaw, roll=0)))

        self.spectators.append(spectator)

    def clear_vehicles(self):
        for i in self.world.get_actors():
            if type(i) == carla.libcarla.Vehicle:
                i.destroy()

    def spawn_vehicles(self):
        self.car_1 = Agent(ssid='car_a')
        self.car_2 = Agent(ssid='car_b')
        self.car_3 = Agent(ssid='car_c')
        self.car_4 = Agent(ssid='car_d')
        self.car_5 = Agent(ssid='car_e')

        self.car_1.connect_carla()
        self.car_2.connect_carla()
        self.car_3.connect_carla()
        self.car_4.connect_carla()
        self.car_5.connect_carla()

        # set up mesh network

        self.car_1.spawn_vehicle(x=-46.8, y=37.0, z=0.50, yaw=-90, template_filter='*charger*')
        self.car_2.spawn_vehicle(x=-13.65, y=-1.00, z=0.50, yaw=180, template_filter='*citroen*')
        self.car_3.spawn_vehicle(x=-13.65, y=-4.10, z=0.50, yaw=180, template_filter='*grandtourer*')
        self.car_4.spawn_vehicle(x=-54.9, y=-34.8, z=0.50, yaw=90, template_filter='*coupe*')
        self.car_5.spawn_vehicle(x=-51.1, y=-34.8, z=0.50, yaw=90, template_filter='*a2*')

        self.car_1.overridespeed = True
        self.car_2.overridespeed = True
        self.car_3.overridespeed = True
        self.car_4.overridespeed = True
        self.car_5.overridespeed = True
        self.car_1.drivingMode = AgentDrivingMode.ASSHOLE
        self.car_2.drivingMode = AgentDrivingMode.MERGING_REQ
        self.car_3.drivingMode = AgentDrivingMode.MERGING_ACT
        self.car_4.drivingMode = AgentDrivingMode.DRIVING_HIGHWAY
        self.car_5.drivingMode = AgentDrivingMode.DRIVING_CITY

        MeshNode.call(self.car_1.portNumber, Request('get_graph_recursive', args=[[]], longRunning=True))

        # self.car_a.driveController.max_steering_angle *= 2
        # self.car_a.purePursuitEndWaypointDist = 1
        # self.car_a._setWaypoints(hardcoded_cop_waypoints())

    def run(self):
        self.world.tick()
        n_tick = 0
        last_tick_t = time()

        self.car_1.waypointList = c1waypoints
        self.car_2.waypointList = c2waypoints
        self.car_3.waypointList = c3waypoints
        self.car_4.waypointList = c4waypoints
        self.car_5.waypointList = c5waypoints

        self.car_1.drivingBehavior = AgentDrivingBehavior.FOLLOW_WAYPOINTS
        self.car_2.drivingBehavior = AgentDrivingBehavior.FOLLOW_WAYPOINTS
        self.car_3.drivingBehavior = AgentDrivingBehavior.FOLLOW_WAYPOINTS
        self.car_4.drivingBehavior = AgentDrivingBehavior.FOLLOW_WAYPOINTS
        self.car_5.drivingBehavior = AgentDrivingBehavior.FOLLOW_WAYPOINTS

        self.car_1.collisionDetection = False
        self.car_2.collisionDetection = False
        self.car_3.collisionDetection = False
        self.car_4.collisionDetection = False
        self.car_5.collisionDetection = False

        while True:
            n_tick += 1
            self.world.tick()
            MeshNode.call(self.car_1.portNumber, Request('tick', args=[], longRunning=True))
            MeshNode.call(self.car_2.portNumber, Request('tick', args=[], longRunning=True))
            MeshNode.call(self.car_3.portNumber, Request('tick', args=[], longRunning=True))
            MeshNode.call(self.car_4.portNumber, Request('tick', args=[], longRunning=True))
            MeshNode.call(self.car_5.portNumber, Request('tick', args=[], longRunning=True))

            def t2l(trns):
                return carla.Location(trns.x, trns.y, 2)

            dbg = self.carla_client.get_world().debug
            c1p = self.car_1.vehiclePose.translation
            c2p = self.car_2.vehiclePose.translation
            c3p = self.car_3.vehiclePose.translation
            c4p = self.car_4.vehiclePose.translation
            c5p = self.car_5.vehiclePose.translation

            dbg.draw_line(t2l(c1p), t2l(c2p), 0.1, carla.Color(255, 0, 0), 0.01)
            dbg.draw_line(t2l(c1p), t2l(c3p), 0.1, carla.Color(255, 0, 0), 0.01)
            dbg.draw_line(t2l(c3p), t2l(c4p), 0.1, carla.Color(255, 0, 0), 0.01)
            dbg.draw_line(t2l(c4p), t2l(c5p), 0.1, carla.Color(255, 0, 0), 0.01)
            dbg.draw_line(t2l(c5p), t2l(c3p), 0.1, carla.Color(255, 0, 0), 0.01)
            dbg.draw_line(t2l(c4p), t2l(c2p), 0.1, carla.Color(255, 0, 0), 0.01)

            dbg.draw_point(t2l(c1p), 0.07, carla.Color(0, 0, 255), 0.01)
            dbg.draw_point(t2l(c2p), 0.07, carla.Color(0, 0, 255), 0.01)
            dbg.draw_point(t2l(c3p), 0.07, carla.Color(0, 0, 255), 0.01)
            dbg.draw_point(t2l(c4p), 0.07, carla.Color(0, 0, 255), 0.01)
            dbg.draw_point(t2l(c5p), 0.07, carla.Color(0, 0, 255), 0.01)

            # dbg.

            low = 30
            high = 10

            if n_tick < 90:
                self.car_1.waypointFollowSpeed = low
            else:
                self.car_1.waypointFollowSpeed = high

            if n_tick < 20:
                self.car_2.waypointFollowSpeed = low
            else:
                self.car_2.waypointFollowSpeed = high

            if n_tick < 90:
                self.car_3.waypointFollowSpeed = low
            else:
                self.car_3.waypointFollowSpeed = high

            if n_tick < 300:
                self.car_4.waypointFollowSpeed = low / 2
            else:
                self.car_4.waypointFollowSpeed = high

            if n_tick < 350:
                self.car_5.waypointFollowSpeed = low / 4
            else:
                self.car_5.waypointFollowSpeed = high

            print(f"Tick: {n_tick}")

            if n_tick == 1000:
                break
            print(f"dt: {time() - last_tick_t}")
            dt = time() - last_tick_t
            if dt < override_dt:
                pass
                sleep(override_dt-dt)
            last_tick_t = time()


c1waypoints = [
    Translation2d(-46.8, 17.0),
    Translation2d(-46.8, -40),
    Translation2d(-46.8, -70),
    Translation2d(-46.8, -100),
]
c2waypoints = [
    Translation2d(-33.65, -1),
    Translation2d(-63.65, -1),
    Translation2d(-83.65, -1),
    Translation2d(-100, -1),
]
c3waypoints = [
    Translation2d(-33.65, -4.10),
    Translation2d(-40.65, -4.10),
    Translation2d(-46.8, -10),
    Translation2d(-46.8, -15),
    Translation2d(-46.8, -30),
    Translation2d(-46.8, -60),
    Translation2d(-46.8, -100),
]
c4waypoints = [
    Translation2d(-54.9, -14.8),
    Translation2d(-54.9, -7.8),
    Translation2d(-41.65, 3.00),
    Translation2d(-33.65, 3.00),
    Translation2d(-20.65, 3.00),
    Translation2d(0., 3.00),
]
c5waypoints = [
    Translation2d(-51.1, -14.8),
    Translation2d(-51.1, 24.8),
    Translation2d(-51.1, 44.8),
    Translation2d(-51.1, 100),
]

if __name__ == '__main__':
    fourway_setup = fourway(ip='localhost')
    # fourway_setup.carla_client.start_recorder('fourway_out.log')
    fourway_setup.setup()
    fourway_setup.run()
