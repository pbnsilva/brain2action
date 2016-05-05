import time
import numpy as np
from base import SerialDevice


class IMUSuit(SerialDevice):

    """
    Joint IDs

                               13
                               |
                               12
                               |
    17 = 16 ==== 15 ==== 14 == 11 == 18 ==== 19 ==== 20 = 21
                               |
                               |
                               10
                               |
                               |
                               0
                               |
                          6 == 1 == 2
                          |         |
                          |         |
                          |         |
                          7         3
                          |         |
                          |         |
                          8         4
                          |         |
                          9         5
    """

    def __init__(self, port=None, from_csv=None, baud_rate=115200):
        super(IMUSuit, self).__init__(port, baud_rate)
        self._calibration_seconds = 60
        self._calibration_vals = []

        # imu index -> joint index
        # self._imu_joints = [
            # 15,     # TODO joints names
        # ]

        if from_csv:
            self._csv_file = open(from_csv, 'r')
            self._imu_joints = [
                15,
                19,
                14,
                18,
                11
            ]

        self._current_imu = 0

    def calibrate(self):
        print 'Calibrating...'
        start = time.time()
        end = start
        while end - start < self._calibration_seconds:
            rot = self.read_rotation()
            if rot:
                rot = rot[1]
                if len(self._calibration_vals) > 0:
                    w = np.mean([v[0] for v in self._calibration_vals])
                    x = np.mean([v[1] for v in self._calibration_vals])
                    y = np.mean([v[2] for v in self._calibration_vals])
                    z = np.mean([v[3] for v in self._calibration_vals])
                    print rot[1] - x, rot[2] - y, rot[3] - z, w
                    print 'AVG: ', x, y, z, w
                self._calibration_vals.append(rot)
            end = time.time()

    def read_rotations(self):
        if self._csv_file:
            rots = []
            vals = self._csv_file.readline().strip().split(',')[1:]
            if not vals:
                return None
            for i in xrange(len(self._imu_joints)):
                rots.append((self._imu_joints[i], map(float, vals[i * 4: i * 4 + 4])))
            return rots
        else:
            vals = self.read_line().split(',')
            if len(vals) == 4:
                self._current_imu = (self._current_imu + 1) % len(self._imu_joints)
                return [self._imu_joints[self._current_imu - 1], map(float, vals)]

    def close(self):
        super(IMUSuit, self).close()
        if self._csv_file:
            self._csv_file.close()
