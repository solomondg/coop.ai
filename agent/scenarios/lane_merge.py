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
        settings.fixed_delta_seconds = 0.01
        self.world.apply_settings(settings)

    def place_spectator(self):
        spectator = [i for i in self.world.get_actors() if i.type_id == 'spectator'][0]
        spectator.set_transform(carla.Transform(carla.Location(x=-205, y=-90, z=15),
                                                carla.Rotation(pitch=-90, yaw=0, roll=0)))

        self.spectators.append(spectator)

    def clear_vehicles(self):
        for i in self.world.get_actors():
            if type(i) == carla.libcarla.Vehicle:
                i.destroy()

    def spawn_vehicles(self):
        self.car_a = Agent(ssid='car_a')
        # self.car_b = Agent(ssid='car_b')
        # self.car_c = Agent(ssid='car_c')

        self.car_a.connect_carla()
        # self.car_b.connect_carla()
        # self.car_c.connect_carla()

        # set up mesh network
        MeshNode.call(self.car_a.portNumber, Request('get_graph_recursive', args=[[]], longRunning=True))

        self.car_a.spawn_vehicle(x=-205, y=-91.75, z=0.1, yaw=0)
        # self.car_b.spawn_vehicle(x=-215, y=-91.75, z=1, yaw=0)
        # self.car_c.spawn_vehicle(x=-215, y=-88.25, z=1, yaw=0)

    def set_waypoints(self):
        # Car A
        car_a_location = self.car_a.get_location
        # self.car_a.set_waypoints

    def run(self):
        n_tick = 0
        rst = False
        self.car_a.velocityReference =8.9408 * 2
        while True:
            n_tick += 1
            self.world.tick()
            MeshNode.call(self.car_a.portNumber, Request('tick', args=[], longRunning=True))
            # MeshNode.call(self.car_b.portNumber, Request('tick', args=[[]], longRunning=True))
            # MeshNode.call(self.car_c.portNumber, Request('tick', args=[[]], longRunning=True))
            car_ref = [i for i in self.world.get_actors() if type(i) is carla.libcarla.Vehicle][0]
            car_loc = car_ref.get_location()
            car_rot = car_ref.get_transform().rotation
            # control = carla.VehicleControl(1, 0, 0, False, False, False, 0)
            # car_ref.apply_control(control)
            if not rst and self.car_a._getCarForwardVelocity() > 0.3:
                rst = True
                n_tick = 0
            # print(
            #     f"t={n_tick / 100}s, car location: {car_loc}, car vel: {self.car_a._getCarForwardVelocity() * 2.23694}mph")
            print(f"t={n_tick / 100}s, car heading: {car_rot}")


def gen_waypoints_straight_x(location):
    pass


if __name__ == '__main__':
    lane_merge_setup = LaneMerge(ip='172.27.51.171')
    lane_merge_setup.setup()
    lane_merge_setup.run()
