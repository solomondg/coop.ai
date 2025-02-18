from time import time, sleep

import carla

from mathutil.Rotation2d import Rotation2d
from services.Agent import Agent, AgentDrivingBehavior, AgentRepresentation, AgentDrivingMode
from services.MeshNode import MeshNode
from apis.Messages import Request
from mathutil.Translation2d import Translation2d

enforce_dt = 0.2

class TBone:
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

    def place_spectator(self, x=35, y=-30, z=35, pitch=-90, yaw=0):
        spectator = [i for i in self.world.get_actors() if i.type_id == 'spectator'][0]
        spectator.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z),
                                                carla.Rotation(pitch=pitch, yaw=yaw, roll=0)))

        self.spectators.append(spectator)

    def clear_vehicles(self):
        for i in self.world.get_actors():
            if type(i) == carla.libcarla.Vehicle:
                i.destroy()

    def spawn_vehicles(self):
        self.car_a = Agent(ssid='car_a')
        self.car_b = Agent(ssid='car_b')

        self.car_a.connect_carla()
        self.car_b.connect_carla()

        # set up mesh network
        MeshNode.call(self.car_a.portNumber, Request('get_graph_recursive', args=[[]], longRunning=True))

        self.car_a.spawn_vehicle(x=45, y=-36.5, z=0.25, yaw=180, template_filter='*charger*')
        self.car_b.spawn_vehicle(x=35, y=-10, z=0.25, yaw=-90, template_filter='*cola*')
        self.car_a.overridespeed=True
        self.car_a.driveController.max_steering_angle *= 2
        #self.car_a.purePursuitEndWaypointDist = 1
        self.car_a._setWaypoints(hardcoded_cop_waypoints())

    def run(self):
        self.world.tick()
        n_tick = 0
        car_b_x_o = self.car_b.carla_vehicle.get_location().x
        last_tick_t = time()
        self.car_b.drivingMode = AgentDrivingMode.DRIVING_CITY
        self.car_a.drivingMode = AgentDrivingMode.ASSHOLE
        while True:
            if n_tick == 170:
                self.car_a._setDrivingBehavior(AgentDrivingBehavior.FOLLOW_WAYPOINTS)
            if n_tick == 100:
                self.car_b._setDrivingBehavior(AgentDrivingBehavior.FOLLOW_WAYPOINTS)
            n_tick += 1
            self.world.tick()
            self.car_a.velocityReference = 15
            self.car_a.waypointFollowSpeed = 15
            self.car_b.velocityReference = 30
            self.car_b.waypointFollowSpeed = 30
            if n_tick >= 170:
                if self.car_b._willCollide(AgentRepresentation.fromAgent(self.car_a)):
                    self.car_b.velocityReference = 0
                    self.car_b.waypointFollowSpeed = 0
                else:
                    self.car_b.velocityReference = 30
                    self.car_b.waypointFollowSpeed = 30
            car_b_loc = self.car_b.vehiclePose
            MeshNode.call(self.car_b.portNumber, Request('tick', args=[], longRunning=True))
            MeshNode.call(self.car_a.portNumber, Request('tick', args=[], longRunning=True))

            print(f"Tick: {n_tick}")
            self.car_b._setWaypoints(gen_waypoints_straight_y(car_b_loc, car_b_x_o))

            if n_tick == 600:
                break
            print(f"dt: {time() - last_tick_t}")
            dt = time() - last_tick_t
            if (dt< enforce_dt):
                print(f"sleeping for: {enforce_dt-dt}")
                sleep(enforce_dt-dt)
            print(f"dt_adj: {time() - last_tick_t}")
            last_tick_t = time()


def gen_waypoints_straight_x(location, original_y, init_dist=10, dist=1, num_points=25):
    waypoints = []
    x = location.x + init_dist
    for i in range(num_points):
        waypoints.append(Translation2d(x + dist, original_y))
        x += dist
    return waypoints

def gen_waypoints_straight_y(location, original_x, init_dist=-10, dist=-1, num_points=25):
    waypoints = []
    y = location.y + init_dist
    for i in range(num_points):
        waypoints.append(Translation2d(original_x, y))
        y += dist
    return waypoints

def hardcoded_cop_waypoints():
    return [
        Translation2d(35, -36),
        Translation2d(27.5, -20),
        Translation2d(27.5, -15),
        Translation2d(27.5, -5),
        Translation2d(27.5, 5),
        Translation2d(30, 15),
    ]


if __name__ == '__main__':
    tbone_setup = TBone(ip='localhost')
    tbone_setup.carla_client.start_recorder('tbone_avoid_out.log')
    tbone_setup.setup()
    tbone_setup.run()
    tbone_setup.carla_client.stop_recorder()

