import numpy as np
from datetime import datetime


class Quaternion:

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.values = np.array([x, y, z, w])

    def as_matrix(self):
        values = self.values
        ret = np.zeros((4, 4), dtype=float)

        ret[0, 0] = 1 - 2 * (values[1] * values[1] + values[2] * values[2])
        ret[0, 1] = 2 * (values[0] * values[1] - values[3] * values[2])
        ret[0, 2] = 2 * (values[3] * values[1] + values[0] * values[2])

        ret[1, 0] = 2 * (values[0] * values[1] + values[3] * values[2])
        ret[1, 1] = 1 - 2 * (values[0] * values[0] + values[2] * values[2])
        ret[1, 2] = 2 * (values[1] * values[2] - values[3] * values[0])

        ret[2, 0] = 2 * (values[0] * values[2] - values[3] * values[1])
        ret[2, 1] = 2 * (values[1] * values[2] + values[3] * values[0])
        ret[2, 2] = 1 - 2 * (values[0] * values[0] + values[1] * values[1])

        ret[3, 3] = 1

        return ret

    def __mul__(self, other):
        ret = Quaternion()
        if isinstance(other, Quaternion):
            left = self.values
            right = other.values
            ret.values[0] = left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1]
            ret.values[1] = left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0]
            ret.values[2] = left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3]
            ret.values[3] = left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2]
        else:
            quat = self.values
            ret.values[0] = quat[0] * other
            ret.values[1] = quat[1] * other
            ret.values[2] = quat[2] * other
            ret.values[3] = quat[3] * other
        return ret

    def __rmul__(self, other):
        return self.__mul__(other)

    def __add__(self, other):
        ret = Quaternion()

        left = self.values
        right = other.values
        ret.values[0] = left[0] + right[0]
        ret.values[1] = left[1] + right[1]
        ret.values[2] = left[2] + right[2]
        ret.values[3] = left[3] + right[3]

        return ret

    def __radd__(self, other):
        return self.__add__(other)

    def get_conjugate(self):
        ret = Quaternion()
        values = self.values
        ret.values[0] = -values[0]
        ret.values[1] = -values[1]
        ret.values[2] = -values[2]
        ret.values[3] = values[3]
        return ret

    def dot_product(self, other):
        values = self.values
        other = other.values
        return values[0] * other[0] + values[1] * other[1] + values[2] * other[2] + values[3] * other[3]

    def normalize(self):
        return np.linalg.norm(self.values)


class Timer:

    def __init__(self):
        self._starts = {}
        self._times = {}

    def start(self, key):
        self._starts[key] = datetime.now()

    def stop(self, key):
        self._times[key] = self._times.get(key, 0) + (datetime.now() - self._starts[key]).microseconds
        del self._starts[key]

    def get(self, key):
        return self._times[key] / float(1000000)


def lerp(from_quaternion, to_quaternion, time):
    ret = from_quaternion * (1 - time) + to_quaternion * time
    ret.normalize()
    return ret


def slerp_no_invert(from_quaternion, to_quaternion, time):
    cos_angle = from_quaternion.dot_product(to_quaternion)
    if cos_angle < 0.95 and cos_angle > -0.95:
        angle = np.arccos(cos_angle)
        sin_invert = 1 / np.sin(angle)
        from_scale = np.sin(angle * (1 - time)) * sin_invert
        to_scale = np.sin(angle * time) * sin_invert
        return from_quaternion * from_scale + to_quaternion * to_scale
    else:
        return lerp(from_quaternion, to_quaternion, time)
