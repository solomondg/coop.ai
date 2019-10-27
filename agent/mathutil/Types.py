import carla
import numpy as np
import math


class Point2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "x: " + str(self.x) + " y: " + str(self.y)

    def __repr__(self):
        return self.__str__()

    def asList(self):
        return [self.x, self.y]

    def asVector(self):
        return Vector2D(self.x, self.y)

    def __sub__(self, other):
        return Point2D(self.x - other.x, self.y - other.y)


class Waypoint:
    cos: float
    sin: float
    theta: float

    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.cos = math.cos(t)
        self.sin = math.sin(t)
        self.theta = t


class Point3D:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def asNDarray(self):
        return np.asarray([self.x, self.y])

    def asNDArray(self):
        return self.asNDarray()

    def normalize(self):
        return toVec2D(self.asNDarray() / (np.linalg.norm(self.asNDarray())))

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def __str__(self):
        return "[" + str(self.x) + " " + str(self.y) + "]"

    def asPoint(self):
        return Point2D(self.x, self.y)

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)


def toVec2D(vec: np.ndarray) -> Vector2D:
    return Vector2D(vec[0], vec[1])

class Vector2d(Vector2D):
    pass

def carlaV3Dot(v1: carla.Vector3D, v2: carla.Vector3D) -> float:
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
