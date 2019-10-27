import math
import numpy as np

from mathutil.Types import Vector2D, toVec2D


class Rotation2d:
    @staticmethod
    def createFromRadians(rad):
        return Rotation2d(math.cos(rad), math.sin(rad))

    @staticmethod
    def createFromDegrees(deg):
        return Rotation2d(math.cos(math.radians(deg)), math.sin(math.radians(deg)))

    rotation = Vector2D(1.0, 0.0)

    def __init__(self, cos=1.0, sin=0.0):
        assert np.linalg.norm([cos, sin]) > 1e-6

        self.rotation = Vector2D(cos, sin).normalize()

    @property
    def cos(self):
        return self.rotation.x

    @property
    def sin(self):
        return self.rotation.y

    @property
    def radians(self):
        return math.atan2(self.sin, self.cos)

    @property
    def degrees(self):
        return math.degrees(self.radians)

    @property
    def normal(self):
        return Rotation2d(self.sin, -self.cos)

    @property
    def antinormal(self):
        return Rotation2d(-self.sin, self.cos)

    def rotateBy(self, other):
        v = rotateVectors(self.rotation, other.rotation)
        return Rotation2d(v.x, v.y)

    @property
    def inverse(self):
        return Rotation2d(self.cos, -self.sin)

    @property
    def theta(self):
        return self.radians


# Rotated v1 by v2
def rotateVectors(v1: Vector2D, v2: Vector2D) -> Vector2D:
    v1 = np.asarray([float(v1.x), float(v1.y)])
    R = np.matrix([
        [float(v2.x), float(-v2.y)],
        [float(v2.y), float(v2.x)]
    ])

    try:
        mtx = R @ v1
    except:
        print("ree")
    return toVec2D(np.asarray(mtx)[0])


if __name__ == "__main__":
    r = Rotation2d(1.0, 0.0)
    m = Rotation2d(1.0, 1.0)
    print(r.rotateBy(m).degrees)
