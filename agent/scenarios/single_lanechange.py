import carla
from services.Agent import Agent, AgentDrivingBehavior
from services.MeshNode import MeshNode
from apis.Messages import Request
from mathutil.Translation2d import Translation2d


class LaneMerge:
    def __init__(self, ip='localhost', port=2000):
        self.carla_client = carla.Client(ip, port)
        self.world = None
        self.car_a = None
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

    def place_spectator(self):
        spectator = [i for i in self.world.get_actors() if i.type_id == 'spectator'][0]
        spectator.set_transform(carla.Transform(carla.Location(x=-205, y=-90, z=45),
                                                carla.Rotation(pitch=-90, yaw=0, roll=0)))

        self.spectators.append(spectator)

    def clear_vehicles(self):
        for i in self.world.get_actors():
            if type(i) == carla.libcarla.Vehicle:
                i.destroy()

    def spawn_vehicles(self):
        self.car_a = Agent(ssid='car_a')

        self.car_a.connect_carla()

        # set up mesh network
        MeshNode.call(self.car_a.portNumber, Request('get_graph_recursive', args=[[]], longRunning=True))

        self.car_a.spawn_vehicle(x=-205, y=-91.25, z=0.1, yaw=0)

    def run(self):
        self.world.tick()
        n_tick = 0
        start_y = self.car_a.carla_vehicle.get_location().y
        lane_width = 4  # m
        lane_change_dist = 10  # m
        lane_change_x = None
        while True:
            n_tick += 1
            self.world.tick()
            MeshNode.call(self.car_a.portNumber, Request('tick', args=[], longRunning=True))
            car_a_loc = self.car_a.vehiclePose

            if n_tick == 100:
                self.car_a._setDrivingBehavior(AgentDrivingBehavior.FOLLOW_WAYPOINTS)
                print("Set to following waypoints mode")
            if n_tick < 500:
                self.car_a._setWaypoints(gen_waypoints_straight_x(car_a_loc, start_y))
            else:
                if lane_change_x is None:
                    print(f"Lane change in {lane_change_dist} m")
                    lane_change_x = car_a_loc.x + lane_change_dist
                self.car_a._setWaypoints(gen_lanechange_waypoints(car_a_loc, start_y, start_y+lane_width, lane_change_x))

            print(f"n_tick: {n_tick}\tangular_vel: {self.car_a.angularVelocityReference}")


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
