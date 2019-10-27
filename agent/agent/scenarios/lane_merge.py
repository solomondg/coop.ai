import carla
from services.Agent import Agent
from services.MeshNode import MeshNode
from apis.Messages import Request


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
        self.world.apply_settings(settings)

    def place_spectator(self):
        spectator = [i for i in self.world.get_actors() if i.type_id == 'spectator'][0]
        spectator.set_transform(carla.Transform(carla.Location(x=-220, y=-90, z=3),
                                                carla.Rotation(pitch=-15, yaw=0, roll=0)))

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

        self.car_a.spawn_vehicle(x=-205, y=-91.75, z=1, yaw=0)
        # self.car_b.spawn_vehicle(x=-215, y=-91.75, z=1, yaw=0)
        # self.car_c.spawn_vehicle(x=-215, y=-88.25, z=1, yaw=0)


    def set_waypoints(self):
        # Car A
        car_a_location = self.car_a.get_location
        # self.car_a.set_waypoints

    def run(self):
        while True:
            MeshNode.call(self.car_a.portNumber, Request('tick', args=[[]], longRunning=True))
            # MeshNode.call(self.car_b.portNumber, Request('tick', args=[[]], longRunning=True))
            # MeshNode.call(self.car_c.portNumber, Request('tick', args=[[]], longRunning=True))

            self.world.tick()


def gen_waypoints_straight_x(location):
    pass


if __name__ == '__main__':
    lane_merge_setup = LaneMerge()
    lane_merge_setup.setup()

    lane_merge_setup.run()
