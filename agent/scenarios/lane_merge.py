import carla

from mathutil.Rotation2d import Rotation2d
from services.Agent import Agent, AgentDrivingBehavior, AgentRepresentation
from services.MeshNode import MeshNode
from apis.Messages import Request
from mathutil.Translation2d import Translation2d


class LaneMerge:
    def __init__(self, ip='localhost', port=2000):
        self.carla_client = carla.Client(ip, port)
        self.world = None
        self.car_a = None
        self.car_b = None
        self.car_c = None
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

    def place_spectator(self, x=-205, y=-90, z=45, pitch=-90, yaw=0):
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
        self.car_c = Agent(ssid='car_c')

        self.car_a.connect_carla()
        self.car_b.connect_carla()
        self.car_c.connect_carla()

        # set up mesh network
        MeshNode.call(self.car_a.portNumber, Request('get_graph_recursive', args=[[]], longRunning=True))

        self.car_a.spawn_vehicle(x=-205, y=-91.75, z=0.1, yaw=0)
        self.car_b.spawn_vehicle(x=-212, y=-91.75, z=0.1, yaw=0)
        self.car_c.spawn_vehicle(x=-205, y=-88.25, z=0.1, yaw=0)

    def run(self):
        self.world.tick()
        n_tick = 0
        # self.car_a.velocityReference =8.9408 * 2
        start_y = self.car_a.carla_vehicle.get_location().y
        start_y_2 = self.car_c.carla_vehicle.get_location().y
        lane_width = 4  # m
        lane_change_dist = 10  # m
        lane_change_x = None
        while True:
            n_tick += 1
            self.world.tick()
            MeshNode.call(self.car_a.portNumber, Request('tick', args=[], longRunning=True))
            MeshNode.call(self.car_b.portNumber, Request('tick', args=[], longRunning=True))
            MeshNode.call(self.car_c.portNumber, Request('tick', args=[], longRunning=True))
            car_a_loc = self.car_a.vehiclePose
            car_b_loc = self.car_b.vehiclePose
            car_c_loc = self.car_c.vehiclePose

            if n_tick == 100:
                self.car_a._setDrivingBehavior(AgentDrivingBehavior.FOLLOW_WAYPOINTS)
                self.car_b._setDrivingBehavior(AgentDrivingBehavior.MAINTAIN_DISTANCE)
                self.car_b.followAxis = Rotation2d(1, 0)
                self.car_b.followTarget = AgentRepresentation.fromAgent(self.car_a)
                self.car_b.followDistance = 8

                self.car_c._setDrivingBehavior(AgentDrivingBehavior.MAINTAIN_DISTANCE)
                self.car_c.followAxis = Rotation2d(1, 0)
                self.car_c.followTarget = AgentRepresentation.fromAgent(self.car_a)
                self.car_c.followDistance = 8
            self.car_a._setWaypoints(gen_waypoints_straight_x(car_a_loc, start_y))
            self.car_b._setWaypoints(gen_waypoints_straight_x(car_b_loc, start_y))
            if n_tick < 500:
                self.car_c._setWaypoints(gen_waypoints_straight_x(car_c_loc, start_y_2))
            else:
                self.car_c._setWaypoints(gen_waypoints_straight_x(car_c_loc, start_y))

            if n_tick == 500:
                print("Enable merge")
                #self.car_b._setDrivingBehavior(AgentDrivingBehavior.MERGING)
                self.car_c._setMerge(AgentRepresentation.fromAgent(self.car_a), AgentRepresentation.fromAgent(self.car_b))
            self.place_spectator(x=car_a_loc.x, y=car_a_loc.y)


def gen_waypoints_straight_x(location, original_y, init_dist=10, dist=1, num_points=25):
    waypoints = []
    x = location.x + init_dist
    for i in range(num_points):
        waypoints.append(Translation2d(x + dist, original_y))
        x += dist
    return waypoints

def gen_lanechange_waypoints(location, original_y, final_y, change_x, init_dist=10, dist=1, num_points=25):
    waypoints = []
    x = location.x + init_dist
    for i in range(num_points):
        x += dist
        y = original_y if x < change_x else final_y
        waypoints.append(Translation2d(x, y))
    return waypoints


if __name__ == '__main__':
    lane_merge_setup = LaneMerge(ip='localhost')
    lane_merge_setup.setup()
    lane_merge_setup.run()
