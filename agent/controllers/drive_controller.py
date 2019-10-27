from typing import Callable, List

import numpy as np

from mathutil.Pose2d import Pose2d
from mathutil.Rotation2d import Rotation2d
from mathutil.Translation2d import Translation2d
from mathutil.Twist2d import Twist2d


class DriveController:
    def __init__(self, wheelbase_length, max_steering_angle=30 * np.pi / 180):
        self.wheelbase_length = wheelbase_length
        self.max_steering_angle = max_steering_angle

    def compute_fk(self, steering_angle, velocity, dt):
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
        print(velocity*dt, dheading)
        return Twist2d(velocity * dt, 0, dheading)

    # def compute_steering_ik(self, target_x, target_y, current_heading):
    def compute_steering_ik(self, target: Translation2d, current_heading: Rotation2d):
        """
        Computes the steering angle to hit the desired target
        :param target_x: X coordinate in m, relative to the car's origin
        :param target_y: Y coordinate in m, relative to the car's origin
        :param current_heading: Current heading, in rads
        :return: Steering angle, in rads
        """

        current_heading = current_heading.inverse

        relative = target.rotateByOrigin(current_heading)

        if relative.y == 0:
            # Protect against divide by 0
            steering_angle = 0
        elif relative.x == 0:
            # Should never happen, if it does, we steer as far as we can
            steering_angle = self.max_steering_angle * relative.y / abs(relative.y)
        else:
            turning_radius = (relative.x ** 2 + relative.y ** 2) / (2 * relative.y)  # Turning radius in m
            steering_angle = np.arctan2(self.wheelbase_length, turning_radius)

            if steering_angle > np.pi / 2:
                steering_angle -= (np.pi)

            # threshold steering angle to our limits
            steering_angle = min(max(steering_angle, -self.max_steering_angle), self.max_steering_angle)

        return steering_angle

    def get_velocity_profile(self, start_vel, end_vel, end_time=5):
        """
        Generates constant acceleration velocity profile
        :param start_vel: Starting velocity, in m/s
        :param end_vel: Ending velocity, in m/s
        :param end_time: Time to reach the ending velocity, in s
        :param total_time: Total amount of time to generate a profile for, in s
        :return: List of tuples, in (time, velocity)
        """

        profile = lambda t: end_vel if t > end_time else start_vel + t * (end_vel - start_vel) / end_time

        return profile

    def lookahead_point(self, waypoint_profile, current_position, current_vel):
        """
        Generates the next point to target, given waypoints and the current vehicle state
        :param waypoint_profile:
        :param current_position:
        :param current_vel:
        :return:
        """

        return None

    def predict(self,
                velocity_profile: Callable,
                waypoints: List[Translation2d],
                pose0: Pose2d,
                predict_dt: float = 0.01,
                predict_end: float = 5.0):
        """
        Predict the car's behavior for the coming time
        """
        # interpolate pure pursuit at predict_dt frequency
        pose = pose0
        pts = waypoints.copy()
        margin = 0.1

        log = []
        for t in np.linspace(0, predict_end, int(predict_end / predict_dt), endpoint=False):

            if (pose.translation - waypoints[0]).l2 <= margin:
                waypoints.pop(0)

            if len(waypoints) == 0:
                break

            vel = velocity_profile(t)
            wp = waypoints[0]

            wp_dp = wp - pose.translation

            steering_angle = self.compute_steering_ik(wp_dp, pose.rotation)
            twist = self.compute_fk(steering_angle, vel, dt=predict_dt)
            #pose = pose.exp(twist)
            pose.rotation = pose.rotation.rotateBy(Rotation2d.createFromRadians(twist.dtheta))
            pose.translation = pose.translation.translateBy(Translation2d(twist.dx,twist.dy).rotateByOrigin(pose.rotation))
            log.append([t, pose])

        return log
