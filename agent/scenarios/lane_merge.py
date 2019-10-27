import carla
from services.Agent import Agent
from services.MeshNode import MeshNode
from apis.Messages import Request


class LaneMerge:
    def __init__(self, ip='localhost', port=2000):
        self.carla_client = carla.Client(ip, port)
        self.world = None
        self.cars = []
        self.spectators = []

    def setup(self):
        self.initialize_carla_world()
        self.place_spectator()
        self.clear_vehicles()
        self.spawn_vehicles()

    def initialize_carla_world(self):
        self.carla_client.set_timeout(10)
        self.world = self.carla_client.load_world('Town05')

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
        car_a = Agent(ssid='car_a')
        car_b = Agent(ssid='car_b')
        car_c = Agent(ssid='car_c')

        car_a.connect_carla()
        car_b.connect_carla()
        car_c.connect_carla()

        # set up mesh network
        MeshNode.call(car_a.portNumber, Request('get_graph_recursive', args=[[]], longRunning=True))

        car_a.spawn_vehicle(x=-205, y=-91.75, z=1, yaw=0)
        car_b.spawn_vehicle(x=-215, y=-91.75, z=1, yaw=0)
        car_c.spawn_vehicle(x=-215, y=-88.25, z=1, yaw=0)

        self.cars.append(car_a)
        self.cars.append(car_b)
        self.cars.append(car_c)


if __name__ == '__main__':
    lane_merge_setup = LaneMerge()
    lane_merge_setup.setup()
