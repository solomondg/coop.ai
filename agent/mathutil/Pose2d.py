import math

from mathutil.Rotation2d import Rotation2d
from mathutil.Translation2d import Translation2d
from mathutil.Twist2d import Twist2d


class Pose2d:
    translation = Translation2d()
    rotation = Rotation2d()

    def __init__(self, trans=Translation2d(), rot=Rotation2d()):
        self.translation = trans
        self.rotation = rot

    @property
    def x(self):
        return self.translation.x

    @property
    def y(self):
        return self.translation.y

    @property
    def cos(self):
        return self.rotation.cos

    @property
    def sin(self):
        return self.rotation.sin

    @property
    def theta(self):
        return self.rotation.radians

    def transformBy(self, other):
        return Pose2d(
            self.translation.translateBy(other.translation),
            self.rotation.rotateBy(other.rotation)
        )

    def relativeTransformBy(self, other):
        return Pose2d(
            self.translation.translateBy(other.translation.rotateByOrigin(self.rotation)),
            self.rotation.rotateBy(other.rotation)
        )

    @property
    def inverse(self):
        return Pose2d(
            self.translation.inverse().rotateByOrigin(self.rotation.inverse), self.rotation.inverse
        )

    @staticmethod
    def exp(delta: Twist2d):
        sin_theta = math.sin(delta.dtheta)
        cos_theta = math.cos(delta.dtheta)
        if (math.fabs(delta.dtheta) < 1E-6):
            s = 1.0 - 1. / 6. * delta.dtheta * delta.dtheta
            c = 0.5 * delta.dtheta
        else:
            s = sin_theta / delta.dtheta
            c = (1.0 - cos_theta) / delta.dtheta

        return Pose2d(
            Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
            Rotation2d(cos_theta, sin_theta)
        )
