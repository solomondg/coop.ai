from mathutil.Rotation2d import Rotation2d, rotateVectors
from mathutil.Types import Vector2D
import numpy as np


class Translation2d:
    position = Vector2D(0, 0)

    @property
    def x(self):
        return self.position.x

    @property
    def y(self):
        return self.position.y

    def __init__(self, x=0.0, y=0.0):
        self.position = Vector2D(x, y)

    @property
    def norm(self):
        return np.linalg.norm(self.position.asNDArray())

    @property
    def l2(self):
        return self.norm

    def translateBy(self, other):
        p = self.position + other.position
        return Translation2d(p.x, p.y)

    @property
    def inverse(self):
        return Translation2d(-self.position.x, -self.position.y)

    def __add__(self, other):
        return self.translateBy(other)

    def __sub__(self, other):
        return self.translateBy(other.inverse)

    def __invert__(self):
        return self.inverse

    def rotateByOrigin(self, rot2d: Rotation2d):
        vec = rotateVectors(self.position, rot2d.rotation)
        return Translation2d(vec.x, vec.y)
