import numpy as np

class DriveController:
    def __init__(self, wheelbase_length, max_steering_angle=30 * np.pi / 180):
        self.wheelbase_length = wheelbase_length
        self.max_steering_angle = max_steering_angle

    def compute_fk(self, steering_angle, velocity, heading, dt):
        """
        Computes the forward kinematics of this vehicle, relative to the current vehicle position
        :param steering_angle: Angle of the front wheels compared to the heading of the car, in rad
        :param velocity: Velocity of the car, in meters
        :param heading: Initial heading of the car, in radians
        :param dt: Time change, in seconds
        :return: x, y, heading in meters/radians, position is relative to to the car's initial position,
        heading is global. positive x is forwards, positive y is left, positive heading is counterclockwise
        """
        dheading = velocity / self.wheelbase_length * np.tan(steering_angle) * dt
        next_heading = heading + dheading
        dxdt = velocity * np.cos(next_heading) * dt
        dydt = velocity * np.sin(next_heading) * dt

        return dxdt, dydt, next_heading

    def compute_steering_ik(self, target_x, target_y, current_heading):
        """
        Computes the steering angle to hit the desired target
        :param target_x: X coordinate in m, relative to the car's origin
        :param target_y: Y coordinate in m, relative to the car's origin
        :param current_heading: Current heading, in rads
        :return: Steering angle, in rads
        """

        current_heading *= -1

        rot_matrix = np.array([
            [np.cos(current_heading), -np.sin(current_heading)],
            [np.sin(current_heading), np.cos(current_heading)]
        ])

        rel_x, rel_y = np.dot(rot_matrix, np.array([target_x, target_y]))

        if rel_y == 0:
            # Protect against divide by 0
            steering_angle = 0
        elif rel_x == 0:
            # Should never happen, if it does, we steer as far as we can
            steering_angle = self.max_steering_angle * rel_y / abs(rel_y)
        else:
            turning_radius = (rel_x ** 2 + rel_y ** 2) / (2 * rel_y)  # Turning radius in m
            steering_angle = np.arctan2(self.wheelbase_length, turning_radius)

            if steering_angle > np.pi / 2:
                steering_angle -= (np.pi)

            # threshold steering angle to our limits
            steering_angle = min(max(steering_angle, -self.max_steering_angle), self.max_steering_angle)

        print(steering_angle)
        return steering_angle

    def get_velocity_profile(self, start_vel, end_vel, end_time=5, total_time=5, dt=0.1):
        """
        Generates constant acceleration velocity profile
        :param start_vel: Starting velocity, in m/s
        :param end_vel: Ending velocity, in m/s
        :param end_time: Time to reach the ending velocity, in s
        :param total_time: Total amount of time to generate a profile for, in s
        :param dt: Step time, in s
        :return: List of tuples, in (time, velocity)
        """
        dvdt = ((end_vel - start_vel) / end_time) * dt

        velocity_profile = [(0, start_vel)]
        current_vel = start_vel

        for t in np.linspace(dt, total_time, int(total_time / dt)):
            if round(t, 5) <= end_time:
                current_vel += dvdt
            velocity_profile.append((t, current_vel))

        return velocity_profile

    def predict(self, velocity_profile, waypoint_profile, initial_heading=0, predict_dt=0.01):
        """
        Predict the car's behavior for the coming time
        """
        # Make sure velocity profile and waypoint profile are solid
        last_vel_time, _ = velocity_profile[-1]
        last_waypoint_time, _, _ = waypoint_profile[-1]
        if last_vel_time != last_waypoint_time or len(velocity_profile) != len(waypoint_profile):
            raise Exception("Velocity and waypoint profile are not in sync")

        # TODO this is making the assumption that the delta times are consistent across the profiles
        last_time = last_waypoint_time
        report_dt = last_waypoint_time / len(waypoint_profile)

        # interpolate pure pursuit at predict_dt frequency
        current_x, current_y = 0, 0
        current_heading = initial_heading
        prediciton_points = []
        for t_raw in np.linspace(0, last_time, int(last_waypoint_time / predict_dt), endpoint=False):
            t = round(t_raw, 5)
            # look for the next projected point
            next_point_index = int(np.floor(t / report_dt))

            _, next_velocity = velocity_profile[next_point_index]
            _, next_x, next_y = waypoint_profile[next_point_index]

            next_x_rel = next_x - current_x
            next_y_rel = next_y - current_y

            steering_angle = self.compute_steering_ik(next_x_rel, next_y_rel, current_heading)
            dx, dy, current_heading = self.compute_fk(steering_angle, next_velocity, current_heading, dt=predict_dt)
            current_x += dx
            current_y += dy

            # TODO find a cleaner way to figure out when to report the data
            if ((t + predict_dt) / report_dt) != next_point_index:
                prediciton_points.append((t, current_x, current_y))

        return prediciton_points
